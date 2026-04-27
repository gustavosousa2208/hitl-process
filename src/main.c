#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree/io-channels.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define APP_NODE DT_PATH(zephyr_user)

#if !DT_NODE_EXISTS(APP_NODE)
#error "Missing zephyr,user node"
#endif

#define PI_F 3.14159265358979323846f

static const struct pwm_dt_spec pwm_left = PWM_DT_SPEC_GET_BY_NAME(APP_NODE, left);
static const struct pwm_dt_spec pwm_right = PWM_DT_SPEC_GET_BY_NAME(APP_NODE, right);

#define ANGLE_DAC_NODE DT_PHANDLE_BY_NAME(APP_NODE, io_channels, angle)
#define ANGLE_DAC_CHANNEL DT_PHA_BY_NAME(APP_NODE, io_channels, angle, output)

#define ANGLE_DAC_RESOLUTION 12U

static const struct device *const angle_dac_dev = DEVICE_DT_GET(ANGLE_DAC_NODE);
static const struct dac_channel_cfg angle_dac_cfg = {
	.channel_id = ANGLE_DAC_CHANNEL,
	.resolution = ANGLE_DAC_RESOLUTION,
	.buffered = true,
};

#define SAMPLE_TIME_MS 50U
#define CAPTURE_TIMEOUT_MS 60U
#define CAPTURE_STALE_LIMIT 5U

#define POT_ADC_CENTER 512.0f
#define POT_ADC_COUNTS_PER_RAD (516.0f / PI_F)
#define ADC_10BIT_MAX 1023.0f

#define ESC_MIN_PULSE_US 1000U
#define ESC_BIDIR_CENTER_PULSE_US 1500U
#define ESC_MAX_PULSE_US 2000U

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

enum plant_run_state {
	PLANT_STATE_STANDBY,
	PLANT_STATE_ARMED,
	PLANT_STATE_RUNNING,
};

struct dashboard_snapshot {
	enum plant_run_state state;
	uint32_t left_pulse_us;
	uint32_t right_pulse_us;
	float left_cmd;
	float right_cmd;
	float angle_rad;
	uint16_t raw_adc;
	uint16_t dac_code;
};

struct timing_stats {
	uint32_t cycles;
	uint32_t deadline_misses;
	uint32_t last_exec_ms;
	uint32_t max_exec_ms;
	uint64_t exec_sum_ms;
	uint32_t last_dt_ms;
	uint32_t min_dt_ms;
	uint32_t max_dt_ms;
};

struct capture_counters {
	uint32_t ok;
	uint32_t eagain;
	uint32_t ebusy;
	uint32_t other;
	int32_t last_ret;
};

struct pwm_input_state {
	uint32_t period_us;
	uint32_t pulse_us;
	float command;
	uint8_t stale_count;
	bool valid;
};

static volatile struct dashboard_snapshot dashboard = {
	.state = PLANT_STATE_STANDBY,
	.left_pulse_us = ESC_MIN_PULSE_US,
	.right_pulse_us = ESC_MIN_PULSE_US,
};
static volatile bool plant_reset_requested;
static volatile bool esc_bidir_enabled;
static volatile bool dashboard_exit_requested;
static volatile bool dashboard_active;
static const struct shell *dashboard_shell;
static struct k_thread dashboard_thread;
static struct k_thread left_capture_thread;
static struct k_thread right_capture_thread;
static struct timing_stats timing = {
	.min_dt_ms = UINT32_MAX,
};
static struct capture_counters capture_left_stats;
static struct capture_counters capture_right_stats;
static struct pwm_input_state left_input = {
	.period_us = 20000U,
	.pulse_us = ESC_MIN_PULSE_US,
	.command = 0.0f,
	.stale_count = CAPTURE_STALE_LIMIT + 1U,
	.valid = false,
};
static struct pwm_input_state right_input = {
	.period_us = 20000U,
	.pulse_us = ESC_MIN_PULSE_US,
	.command = 0.0f,
	.stale_count = CAPTURE_STALE_LIMIT + 1U,
	.valid = false,
};
static struct k_spinlock input_lock;
K_THREAD_STACK_DEFINE(dashboard_thread_stack, 2048);
K_THREAD_STACK_DEFINE(left_capture_thread_stack, 1024);
K_THREAD_STACK_DEFINE(right_capture_thread_stack, 1024);

static const struct plant_model model = {
	.b = 0.25e-1f,
	.m = 0.70f,
	.l = 0.45f,
	.g = 9.81f,
	.x = 0.012f,
	.j = (0.70f * (2.0f * 0.45f) * (2.0f * 0.45f)) / 12.0f,
	.gain = 0.0016f * ((0.70f * (2.0f * 0.45f) * (2.0f * 0.45f)) / 12.0f) * 0.9f * 10000.0f,
};

static const char *run_state_name(enum plant_run_state state)
{
	switch (state) {
	case PLANT_STATE_STANDBY:
		return "standby";
	case PLANT_STATE_ARMED:
		return "armed";
	case PLANT_STATE_RUNNING:
		return "running";
	default:
		return "unknown";
	}
}

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
	uint32_t clamped = CLAMP(pulse_us, ESC_MIN_PULSE_US, ESC_MAX_PULSE_US);

	if (esc_bidir_enabled) {
		if (clamped == ESC_BIDIR_CENTER_PULSE_US) {
			return 0.0f;
		}

		if (clamped > ESC_BIDIR_CENTER_PULSE_US) {
			return (float)(clamped - ESC_BIDIR_CENTER_PULSE_US) /
			       (float)(ESC_MAX_PULSE_US - ESC_BIDIR_CENTER_PULSE_US);
		}

		return -((float)(ESC_BIDIR_CENTER_PULSE_US - clamped) /
			 (float)(ESC_BIDIR_CENTER_PULSE_US - ESC_MIN_PULSE_US));
	}

	if (clamped <= ESC_MIN_PULSE_US) {
		return 0.0f;
	}

	return (float)(clamped - ESC_MIN_PULSE_US) /
	       (float)(ESC_MAX_PULSE_US - ESC_MIN_PULSE_US);
}

static bool update_motor_command(const struct pwm_dt_spec *spec,
				 uint32_t *period_us,
				 uint32_t *pulse_us,
				 float *command,
				 uint8_t *stale_count,
				 struct capture_counters *counters)
{
	int ret = capture_pwm_usec(spec, period_us, pulse_us);
	counters->last_ret = ret;

	if (ret == -EBUSY) {
		counters->ebusy++;
		pwm_disable_capture(spec->dev, spec->channel);
		*command = 0.0f;
		*stale_count = CAPTURE_STALE_LIMIT + 1U;
		return false;
	}

	if (ret == -EAGAIN) {
		counters->eagain++;
		if (*stale_count < UINT8_MAX) {
			(*stale_count)++;
		}

		if (*stale_count > CAPTURE_STALE_LIMIT) {
			*command = 0.0f;
			return false;
		}

		return true;
	}

	if (ret != 0) {
		counters->other++;
		*command = 0.0f;
		*stale_count = CAPTURE_STALE_LIMIT + 1U;
		return false;
	}

	counters->ok++;
	*command = esc_pulse_to_command(*pulse_us);
	*stale_count = 0U;
	return true;
}

static void reset_pwm_input_state(struct pwm_input_state *input)
{
	input->period_us = 20000U;
	input->pulse_us = ESC_MIN_PULSE_US;
	input->command = 0.0f;
	input->stale_count = CAPTURE_STALE_LIMIT + 1U;
	input->valid = false;
}

static void pwm_capture_thread_fn(void *arg1, void *arg2, void *arg3)
{
	const struct pwm_dt_spec *spec = arg1;
	struct pwm_input_state *input = arg2;
	struct capture_counters *counters = arg3;
	uint32_t period_us = 20000U;
	uint32_t pulse_us = ESC_MIN_PULSE_US;
	float command = 0.0f;
	uint8_t stale_count = CAPTURE_STALE_LIMIT + 1U;

	while (1) {
		bool valid = update_motor_command(spec, &period_us, &pulse_us, &command,
						  &stale_count, counters);
		k_spinlock_key_t key = k_spin_lock(&input_lock);
		input->period_us = period_us;
		input->pulse_us = pulse_us;
		input->command = command;
		input->stale_count = stale_count;
		input->valid = valid;
		k_spin_unlock(&input_lock, key);
	}
}

static void reset_capture_stats(void)
{
	capture_left_stats.ok = 0U;
	capture_left_stats.eagain = 0U;
	capture_left_stats.ebusy = 0U;
	capture_left_stats.other = 0U;
	capture_left_stats.last_ret = 0;

	capture_right_stats.ok = 0U;
	capture_right_stats.eagain = 0U;
	capture_right_stats.ebusy = 0U;
	capture_right_stats.other = 0U;
	capture_right_stats.last_ret = 0;
}

static enum plant_run_state detect_run_state(bool left_valid, bool right_valid,
					     const struct plant_state *state)
{
	if (!left_valid || !right_valid) {
		return PLANT_STATE_STANDBY;
	}

	if ((state->left_cmd != 0.0f) || (state->right_cmd != 0.0f)) {
		return PLANT_STATE_RUNNING;
	}

	return PLANT_STATE_ARMED;
}

static void print_run_state(enum plant_run_state state)
{
	printk("%s\n", run_state_name(state));
}

static void step_plant(struct plant_state *state, float dt_s)
{
	const float torque_term = model.gain * (state->left_cmd - state->right_cmd);
	const float damping_term = model.b * state->omega_rad_s;
	const float gravity_term = model.m * model.g * model.x * sinf(state->theta_rad);
	const float alpha_rad_s2 = (torque_term - damping_term - gravity_term) / model.j;

	state->omega_rad_s += alpha_rad_s2 * dt_s;
	state->theta_rad += state->omega_rad_s * dt_s;
	state->theta_rad = CLAMP(state->theta_rad, -PI_F, PI_F);
}

static void reset_timing_stats(void)
{
	timing.cycles = 0U;
	timing.deadline_misses = 0U;
	timing.last_exec_ms = 0U;
	timing.max_exec_ms = 0U;
	timing.exec_sum_ms = 0U;
	timing.last_dt_ms = 0U;
	timing.min_dt_ms = UINT32_MAX;
	timing.max_dt_ms = 0U;
}

static void update_timing_stats(uint32_t exec_ms, uint32_t dt_ms, bool deadline_missed)
{
	timing.cycles++;
	timing.last_exec_ms = exec_ms;
	timing.max_exec_ms = MAX(timing.max_exec_ms, exec_ms);
	timing.exec_sum_ms += exec_ms;
	timing.last_dt_ms = dt_ms;
	timing.min_dt_ms = MIN(timing.min_dt_ms, dt_ms);
	timing.max_dt_ms = MAX(timing.max_dt_ms, dt_ms);

	if (deadline_missed) {
		timing.deadline_misses++;
	}
}

static uint16_t angle_to_dac_code(float theta_rad)
{
	float clamped_adc = CLAMP(POT_ADC_CENTER + theta_rad * POT_ADC_COUNTS_PER_RAD,
				  0.0f, ADC_10BIT_MAX);
	float dac_counts = (clamped_adc / ADC_10BIT_MAX) * 4095.0f;

	return (uint16_t)CLAMP((int)lroundf(dac_counts), 0, 4095);
}

static uint16_t angle_to_raw_adc(float theta_rad)
{
	float adc_counts = POT_ADC_CENTER + theta_rad * POT_ADC_COUNTS_PER_RAD;

	return (uint16_t)CLAMP((int)lroundf(adc_counts), 0, (int)ADC_10BIT_MAX);
}

static void dashboard_bypass_cb(const struct shell *sh, uint8_t *data, size_t len, void *user_data)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	ARG_UNUSED(user_data);

	dashboard_exit_requested = true;
}

static void dashboard_thread_fn(void *arg1, void *arg2, void *arg3)
{
	const struct shell *sh = arg1;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (!dashboard_exit_requested) {
		int32_t angle_cdeg = (int32_t)lroundf((dashboard.angle_rad * 18000.0f) / PI_F);
		int32_t left_pct_tenths = (int32_t)lroundf(dashboard.left_cmd * 1000.0f);
		int32_t right_pct_tenths = (int32_t)lroundf(dashboard.right_cmd * 1000.0f);

		shell_fprintf(sh, SHELL_NORMAL,
			      "\033[2J\033[H"
			      "HIL dashboard\n"
			      "state: %s\n"
			      "esc mode   : %s\n"
			      "left input : %4u us  %4d.%1u %%\n"
			      "right input: %4u us  %4d.%1u %%\n"
			      "raw adc    : %4u\n"
			      "angle      : %4d.%02d deg\n"
			      "dac value  : %4u\n",
			      run_state_name(dashboard.state),
			      esc_bidir_enabled ? "bidir" : "forward",
			      dashboard.left_pulse_us,
			      left_pct_tenths / 10,
			      (left_pct_tenths < 0) ? -(left_pct_tenths % 10) : (left_pct_tenths % 10),
			      dashboard.right_pulse_us,
			      right_pct_tenths / 10,
			      (right_pct_tenths < 0) ? -(right_pct_tenths % 10) : (right_pct_tenths % 10),
			      dashboard.raw_adc,
			      angle_cdeg / 100,
			      (angle_cdeg < 0) ? -(angle_cdeg % 100) : (angle_cdeg % 100),
			      dashboard.dac_code);
		k_msleep(200);
	}

	shell_set_bypass(sh, NULL, NULL);
	shell_print(sh, "dashboard stopped");
	dashboard_active = false;
	dashboard_shell = NULL;
}

static void reset_plant_state(struct plant_state *state)
{
	state->theta_rad = 0.0f;
	state->omega_rad_s = 0.0f;
	state->left_cmd = 0.0f;
	state->right_cmd = 0.0f;
	state->left_period_us = 20000U;
	state->right_period_us = 20000U;
	state->left_pulse_us = ESC_MIN_PULSE_US;
	state->right_pulse_us = ESC_MIN_PULSE_US;
}

static int cmd_dashboard(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (dashboard_active) {
		shell_error(sh, "dashboard already running");
		return -EBUSY;
	}

	shell_print(sh, "Press any key to stop");
	dashboard_active = true;
	dashboard_shell = sh;
	dashboard_exit_requested = false;
	shell_set_bypass(sh, dashboard_bypass_cb, NULL);

	k_thread_create(&dashboard_thread, dashboard_thread_stack,
			K_THREAD_STACK_SIZEOF(dashboard_thread_stack),
			dashboard_thread_fn, (void *)sh, NULL, NULL,
			K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);

	return 0;
}

static int cmd_plant_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	plant_reset_requested = true;
	shell_print(sh, "plant reset requested");

	return 0;
}

static int cmd_plant_esc_set_bidir_on(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	esc_bidir_enabled = true;
	shell_print(sh, "ESC bidirectional mode enabled");

	return 0;
}

static int cmd_plant_esc_set_bidir_off(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	esc_bidir_enabled = false;
	shell_print(sh, "ESC bidirectional mode disabled");

	return 0;
}

static int cmd_plant_timing(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t avg_exec_ms;
	uint32_t last_util_tenths;
	uint32_t avg_util_tenths;
	uint32_t max_util_tenths;
	int32_t worst_slack_ms;

	if ((argc > 1) && (strcmp(argv[1], "reset") == 0)) {
		reset_timing_stats();
		shell_print(sh, "timing stats reset");
		return 0;
	}

	if (timing.cycles == 0U) {
		shell_print(sh, "timing: no samples yet");
		return 0;
	}

	avg_exec_ms = (uint32_t)(timing.exec_sum_ms / timing.cycles);
	last_util_tenths = (timing.last_exec_ms * 1000U) / SAMPLE_TIME_MS;
	avg_util_tenths = (avg_exec_ms * 1000U) / SAMPLE_TIME_MS;
	max_util_tenths = (timing.max_exec_ms * 1000U) / SAMPLE_TIME_MS;
	worst_slack_ms = (int32_t)SAMPLE_TIME_MS - (int32_t)timing.max_exec_ms;

	shell_print(sh,
			"timing: cycles=%u misses=%u last_exec=%u ms avg_exec=%u ms max_exec=%u ms last_dt=%u ms min_dt=%u ms max_dt=%u ms",
			timing.cycles,
			timing.deadline_misses,
			timing.last_exec_ms,
			avg_exec_ms,
			timing.max_exec_ms,
			timing.last_dt_ms,
			timing.min_dt_ms,
			timing.max_dt_ms);
	shell_print(sh,
			"budget: last=%u.%u%% avg=%u.%u%% max=%u.%u%% worst_slack=%d ms",
			last_util_tenths / 10U,
			last_util_tenths % 10U,
			avg_util_tenths / 10U,
			avg_util_tenths % 10U,
			max_util_tenths / 10U,
			max_util_tenths % 10U,
			worst_slack_ms);

	return 0;
}

static int cmd_plant_capture(const struct shell *sh, size_t argc, char **argv)
{
	if ((argc > 1) && (strcmp(argv[1], "reset") == 0)) {
		reset_capture_stats();
		shell_print(sh, "capture stats reset");
		return 0;
	}

	shell_print(sh,
			"capture left : ok=%u eagain=%u ebusy=%u other=%u last_ret=%d",
			capture_left_stats.ok,
			capture_left_stats.eagain,
			capture_left_stats.ebusy,
			capture_left_stats.other,
			capture_left_stats.last_ret);
	shell_print(sh,
			"capture right: ok=%u eagain=%u ebusy=%u other=%u last_ret=%d",
			capture_right_stats.ok,
			capture_right_stats.eagain,
			capture_right_stats.ebusy,
			capture_right_stats.other,
			capture_right_stats.last_ret);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_plant_esc_set_bidir,
	SHELL_CMD(on, NULL, "Enable bidirectional ESC input mode", cmd_plant_esc_set_bidir_on),
	SHELL_CMD(off, NULL, "Disable bidirectional ESC input mode", cmd_plant_esc_set_bidir_off),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_plant_esc_set,
	SHELL_CMD(bidir, &sub_plant_esc_set_bidir, "Set ESC bidirectional mode", NULL),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_plant_esc,
	SHELL_CMD(set, &sub_plant_esc_set, "ESC settings", NULL),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_plant,
	SHELL_CMD(dashboard, NULL, "Show live plant dashboard; press any key to stop",
		  cmd_dashboard),
	SHELL_CMD(reset, NULL, "Reset plant state to zero", cmd_plant_reset),
	SHELL_CMD(timing, NULL, "Show timing stats; use 'plant timing reset' to clear",
		  cmd_plant_timing),
	SHELL_CMD(capture, NULL, "Show capture stats; use 'plant capture reset' to clear",
		  cmd_plant_capture),
	SHELL_CMD(esc, &sub_plant_esc, "ESC configuration commands", NULL),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(plant, &sub_plant, "Plant control commands", NULL);

int main(void)
{
	struct plant_state state = {
		.left_period_us = 2040U,
		.right_period_us = 2040U,
		.left_pulse_us = ESC_MIN_PULSE_US,
		.right_pulse_us = ESC_MIN_PULSE_US,
	};
	enum plant_run_state current_state = PLANT_STATE_STANDBY;
	int64_t last_step_ms;
	int64_t next_release_ms;
	int ret;

	if (!pwm_is_ready_dt(&pwm_left) || !pwm_is_ready_dt(&pwm_right)) {
		printk("PWM capture device not ready\n");
		return 0;
	}

	if (!device_is_ready(angle_dac_dev)) {
		printk("DAC device not ready\n");
		return 0;
	}

	ret = dac_channel_setup(angle_dac_dev, &angle_dac_cfg);
	if (ret != 0) {
		printk("DAC channel setup failed: %d\n", ret);
		return 0;
	}

	ret = dac_write_value(angle_dac_dev, angle_dac_cfg.channel_id,
			      angle_to_dac_code(0.0f));
	if (ret != 0) {
		printk("Initial DAC write failed: %d\n", ret);
		return 0;
	}

	printk("HIL plant started on %s\n", CONFIG_BOARD_TARGET);
	printk("Console: board default UART @ 115200\n");
	printk("PWM in: left=PB4 right=PA0, DAC out: PA4\n");
	print_run_state(current_state);

	reset_timing_stats();
	reset_capture_stats();
	reset_pwm_input_state(&left_input);
	reset_pwm_input_state(&right_input);
	k_thread_create(&left_capture_thread, left_capture_thread_stack,
			K_THREAD_STACK_SIZEOF(left_capture_thread_stack),
			pwm_capture_thread_fn, (void *)&pwm_left, (void *)&left_input,
			(void *)&capture_left_stats, K_LOWEST_APPLICATION_THREAD_PRIO,
			0, K_NO_WAIT);
	k_thread_create(&right_capture_thread, right_capture_thread_stack,
			K_THREAD_STACK_SIZEOF(right_capture_thread_stack),
			pwm_capture_thread_fn, (void *)&pwm_right, (void *)&right_input,
			(void *)&capture_right_stats, K_LOWEST_APPLICATION_THREAD_PRIO,
			0, K_NO_WAIT);
	last_step_ms = k_uptime_get();
	next_release_ms = last_step_ms + SAMPLE_TIME_MS;

	while (1) {
		uint16_t raw_adc;
		uint16_t dac_code;
		int64_t loop_start_ms = k_uptime_get();
		int64_t now_ms;
		int64_t dt_ms;
		float dt_s;
		bool left_valid;
		bool right_valid;

		if (plant_reset_requested) {
			reset_plant_state(&state);
			k_spinlock_key_t key = k_spin_lock(&input_lock);
			reset_pwm_input_state(&left_input);
			reset_pwm_input_state(&right_input);
			k_spin_unlock(&input_lock, key);
			current_state = PLANT_STATE_STANDBY;
			plant_reset_requested = false;
			print_run_state(current_state);
		}

		k_spinlock_key_t key = k_spin_lock(&input_lock);
		state.left_period_us = left_input.period_us;
		state.left_pulse_us = left_input.pulse_us;
		state.left_cmd = left_input.command;
		left_valid = left_input.valid;
		state.right_period_us = right_input.period_us;
		state.right_pulse_us = right_input.pulse_us;
		state.right_cmd = right_input.command;
		right_valid = right_input.valid;
		k_spin_unlock(&input_lock, key);

		enum plant_run_state next_state = detect_run_state(left_valid, right_valid, &state);

		if (next_state != current_state) {
			current_state = next_state;
			print_run_state(current_state);
		}

		now_ms = k_uptime_get();
		dt_ms = now_ms - last_step_ms;
		if (dt_ms <= 0) {
			dt_ms = SAMPLE_TIME_MS;
		}
		dt_s = (float)dt_ms / 1000.0f;
		step_plant(&state, dt_s);
		last_step_ms = now_ms;

		raw_adc = angle_to_raw_adc(state.theta_rad);
		dac_code = angle_to_dac_code(state.theta_rad);

		ret = dac_write_value(angle_dac_dev, angle_dac_cfg.channel_id,
				      dac_code);
		if (ret != 0) {
			printk("DAC write failed: %d\n", ret);
		}

		dashboard.state = current_state;
		dashboard.left_pulse_us = state.left_pulse_us;
		dashboard.right_pulse_us = state.right_pulse_us;
		dashboard.left_cmd = state.left_cmd;
		dashboard.right_cmd = state.right_cmd;
		dashboard.angle_rad = state.theta_rad;
		dashboard.raw_adc = raw_adc;
		dashboard.dac_code = dac_code;

		now_ms = k_uptime_get();
		bool deadline_missed = (now_ms > next_release_ms);
		update_timing_stats((uint32_t)(now_ms - loop_start_ms), (uint32_t)dt_ms,
				    deadline_missed);

		if (!deadline_missed) {
			k_msleep((uint32_t)(next_release_ms - now_ms));
		}

		now_ms = k_uptime_get();
		next_release_ms += SAMPLE_TIME_MS;
		if (next_release_ms <= now_ms) {
			next_release_ms = now_ms + SAMPLE_TIME_MS;
		}
	}

	return 0;
}
