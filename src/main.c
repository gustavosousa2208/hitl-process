#include <errno.h>
#include <math.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define APP_NODE DT_PATH(zephyr_user)

#if !DT_NODE_EXISTS(APP_NODE)
#error "Missing zephyr,user node"
#endif

#define PI_F 3.14159265358979323846f

static const struct pwm_dt_spec pwm_left = PWM_DT_SPEC_GET_BY_NAME(APP_NODE, left);
static const struct pwm_dt_spec pwm_right = PWM_DT_SPEC_GET_BY_NAME(APP_NODE, right);
static const struct dac_dt_spec angle_dac = DAC_DT_SPEC_GET_BY_NAME(APP_NODE, angle);

#define SAMPLE_TIME_MS 50U
#define CAPTURE_TIMEOUT_MS 60U

#define POT_ADC_CENTER 512.0f
#define POT_ADC_COUNTS_PER_RAD (516.0f / PI_F)
#define ADC_10BIT_MAX 1023.0f

#define ESC_MIN_PULSE_US 1000U
#define ESC_MAX_PULSE_US 1500U
#define ESC_INIT_MAX_PULSE_US 2000U

struct plant_model {
	float b;
	float m;
	float l;
	float g;
	float x;
	float j;
	float gain;
};

struct plant_state {
	float theta_rad;
	float omega_rad_s;
	float left_cmd;
	float right_cmd;
	uint32_t left_period_us;
	uint32_t right_period_us;
	uint32_t left_pulse_us;
	uint32_t right_pulse_us;
};

static const struct plant_model model = {
	.b = 0.25e-1f,
	.m = 0.70f,
	.l = 0.45f,
	.g = 9.81f,
	.x = 0.012f,
	.j = (0.70f * (2.0f * 0.45f) * (2.0f * 0.45f)) / 12.0f,
	.gain = 0.0016f * ((0.70f * (2.0f * 0.45f) * (2.0f * 0.45f)) / 12.0f) * 0.9f * 10000.0f,
};

static int capture_pwm_usec(const struct pwm_dt_spec *spec, uint32_t *period_us, uint32_t *pulse_us)
{
	uint64_t period = 0U;
	uint64_t pulse = 0U;
	int ret = pwm_capture_usec(spec->dev, spec->channel,
				   PWM_CAPTURE_TYPE_BOTH | PWM_CAPTURE_MODE_SINGLE | spec->flags,
				   &period, &pulse, Z_TIMEOUT_MS(CAPTURE_TIMEOUT_MS));

	if (ret == 0) {
		*period_us = (uint32_t)period;
		*pulse_us = (uint32_t)pulse;
	}

	return ret;
}

static float esc_pulse_to_command(uint32_t pulse_us)
{
	uint32_t clamped = CLAMP(pulse_us, ESC_MIN_PULSE_US, ESC_INIT_MAX_PULSE_US);

	if (clamped <= ESC_MIN_PULSE_US) {
		return 0.0f;
	}

	if (clamped >= ESC_MAX_PULSE_US) {
		return 1.0f;
	}

	return (float)(clamped - ESC_MIN_PULSE_US) /
	       (float)(ESC_MAX_PULSE_US - ESC_MIN_PULSE_US);
}

static void update_motor_command(const struct pwm_dt_spec *spec,
				 uint32_t *period_us,
				 uint32_t *pulse_us,
				 float *command,
				 const char *name)
{
	int ret = capture_pwm_usec(spec, period_us, pulse_us);

	if (ret == -EBUSY) {
		pwm_disable_capture(spec->dev, spec->channel);
		printk("%s capture restarted after busy state\n", name);
		return;
	}

	if (ret == -EAGAIN) {
		return;
	}

	if (ret != 0) {
		printk("%s capture error: %d\n", name, ret);
		return;
	}

	*command = esc_pulse_to_command(*pulse_us);
}

static void step_plant(struct plant_state *state)
{
	const float torque_term = model.gain * (state->left_cmd - state->right_cmd);
	const float damping_term = model.b * state->omega_rad_s;
	const float gravity_term = model.m * model.g * model.x * sinf(state->theta_rad);
	const float alpha_rad_s2 = (torque_term - damping_term - gravity_term) / model.j;
	const float dt_s = SAMPLE_TIME_MS / 1000.0f;

	state->omega_rad_s += alpha_rad_s2 * dt_s;
	state->theta_rad += state->omega_rad_s * dt_s;
	state->theta_rad = CLAMP(state->theta_rad, -PI_F, PI_F);
}

static uint16_t angle_to_dac_code(float theta_rad)
{
	float adc_counts = POT_ADC_CENTER + theta_rad * POT_ADC_COUNTS_PER_RAD;
	float clamped_adc = CLAMP(adc_counts, 0.0f, ADC_10BIT_MAX);
	float dac_counts = (clamped_adc / ADC_10BIT_MAX) * 4095.0f;

	return (uint16_t)CLAMP((int)lroundf(dac_counts), 0, 4095);
}

static uint32_t angle_to_millivolts(float theta_rad)
{
	uint16_t dac_code = angle_to_dac_code(theta_rad);

	return ((uint32_t)dac_code * 3300U) / 4095U;
}

int main(void)
{
	struct plant_state state = {
		.left_period_us = 2040U,
		.right_period_us = 2040U,
		.left_pulse_us = ESC_MIN_PULSE_US,
		.right_pulse_us = ESC_MIN_PULSE_US,
	};
	uint32_t loop_count = 0U;
	int ret;

	if (!pwm_is_ready_dt(&pwm_left) || !pwm_is_ready_dt(&pwm_right)) {
		printk("PWM capture device not ready\n");
		return 0;
	}

	if (!device_is_ready(angle_dac.dev)) {
		printk("DAC device not ready\n");
		return 0;
	}

	ret = dac_channel_setup_dt(&angle_dac);
	if (ret != 0) {
		printk("DAC channel setup failed: %d\n", ret);
		return 0;
	}

	ret = dac_write_value_dt(&angle_dac, angle_to_dac_code(0.0f));
	if (ret != 0) {
		printk("Initial DAC write failed: %d\n", ret);
		return 0;
	}

	printk("HIL plant started on %s\n", CONFIG_BOARD_TARGET);
	printk("Console: board default UART @ 115200\n");
	printk("PWM in: left=PB4 right=PB14, DAC out: PA4\n");

	while (1) {
		update_motor_command(&pwm_left, &state.left_period_us, &state.left_pulse_us,
				     &state.left_cmd, "left");
		update_motor_command(&pwm_right, &state.right_period_us, &state.right_pulse_us,
				     &state.right_cmd, "right");

		step_plant(&state);

		ret = dac_write_value_dt(&angle_dac, angle_to_dac_code(state.theta_rad));
		if (ret != 0) {
			printk("DAC write failed: %d\n", ret);
		}

		if ((loop_count % 20U) == 0U) {
			int32_t angle_mdeg = (int32_t)lroundf((state.theta_rad * 180000.0f) /
							      PI_F);

			printk("l=%uus r=%uus angle=%d mdeg out=%u mV\n",
			       state.left_pulse_us, state.right_pulse_us,
			       angle_mdeg, angle_to_millivolts(state.theta_rad));
		}

		loop_count++;
		k_msleep(SAMPLE_TIME_MS);
	}

	return 0;
}
