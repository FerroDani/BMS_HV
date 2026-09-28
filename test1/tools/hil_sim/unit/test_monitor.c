/* Unit tests of Core/Src/bms_monitor.c on the host (gcc), with a fake AFE. */
#include <stdio.h>
#include <string.h>
#include "bms_afe.h"
#include "bms_monitor.h"

int stub_ams_pin, stub_err_led;
uint8_t ams_error;
static bms_afe_t fake;
const bms_afe_t *bms_afe_get(void) { return &fake; }
int16_t bms_afe_ntc_dc(uint16_t raw) { /* same curve as bms_afe.c */
    float v = (float)raw * 0.000089f, v2 = v * v;
    float t = 121.77018530814999f - 92.86284471272114f * v + 39.33876760742037f * v2 - 8.798397291741638f * v2 * v + 0.7020187110716422f * v2 * v2;
    float dc = t * 10.0f; if (dc > 3000.0f) dc = 3000.0f; if (dc < -1000.0f) dc = -1000.0f;
    return (int16_t)(dc >= 0.0f ? dc + 0.5f : dc - 0.5f);
}

static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint16_t raw_mv(int mv) { return (uint16_t)((mv * 1000 + 44) / 89); }

/* all slaves: good cells 3700 mV, NTC 2500 mV (25 C) */
static void reset_all(uint32_t t0) {
    memset(&fake, 0, sizeof(fake));
    ams_error = 0; stub_ams_pin = 0; stub_err_led = 0;
    bms_monitor_init(t0);
}
static void sample_cells(uint32_t t, int cell_mv_slave1_c0) {
    for (unsigned i = 0; i < BMS_N_SLAVES; i++) {
        for (unsigned c = 0; c < BMS_CELLS_PER_SLAVE; c++) fake.slave[i].cell_raw[c] = raw_mv(3700);
        fake.slave[i].last_ok_ms = t; fake.slave[i].cell_seq++;
    }
    fake.slave[0].cell_raw[0] = raw_mv(cell_mv_slave1_c0);
}
static void sample_temps(uint32_t t, int ntc_mv_slave1_g0) {
    for (unsigned i = 0; i < BMS_N_SLAVES; i++) {
        for (unsigned g = 0; g < BMS_GPIO_PER_SLAVE; g++) fake.slave[i].gpio_raw[g] = raw_mv(2500);
        fake.slave[i].last_gpio_ok_ms = t; fake.slave[i].gpio_seq++;
    }
    fake.slave[0].gpio_raw[0] = raw_mv(ntc_mv_slave1_g0);
}
/* runs the monitor every 10 ms and the measurement every 100 ms (temps every 200 ms) from t0 to t1;
 * cell(t) / ntc(t) give the value of the injected channel; returns the latch time or 0 */
static uint32_t simulate(uint32_t t0, uint32_t t1, int (*cell)(uint32_t), int (*ntc)(uint32_t)) {
    uint32_t latched = 0;
    for (uint32_t t = t0; t != t1; t += 10) {
        if ((t - t0) % 100 == 0) {
            sample_cells(t, cell(t));
            if ((t - t0) % 200 == 0) sample_temps(t, ntc(t));
        }
        bms_monitor_update(t);
        if (!latched && bms_monitor_get()->ams_error) latched = t;
    }
    return latched;
}
static uint32_t T_ON, T_OFF;
static int cell_ok(uint32_t t) { (void)t; return 3700; }
static int ntc_ok(uint32_t t) { (void)t; return 2500; }
static int active(uint32_t t) { return (int32_t)(t - T_ON) >= 0 && (int32_t)(t - T_OFF) < 0; }
static int cell_ov_window(uint32_t t) { return active(t) ? 4250 : 3700; }
static int cell_uv_window(uint32_t t) { return active(t) ? 2600 : 3700; }
static int ntc_ot_window(uint32_t t) { return active(t) ? 900 : 2500; }
static int ntc_open_window(uint32_t t) { return active(t) ? 4760 : 2500; }
#define NEVER (T_ON + 100000u)

int main(void) {
    uint32_t base, lt;

    /* 1. reaction time for a real over-voltage is <= FAULT_TIME_VOLTAGE + 10 ms from the onset,
     *    for every phase of the onset with respect to the measurement grid */
    uint32_t worst = 0, best = 0xFFFFFFFF;
    for (uint32_t ph = 0; ph < 100; ph += 5) {
        reset_all(0);
        T_ON = 1000 + ph; T_OFF = NEVER;
        lt = simulate(0, 3000, cell_ov_window, ntc_ok);
        CHECK(lt != 0, "OV latched (phase %u)", ph);
        CHECK(bms_monitor_get()->faults == BMS_FAULT_CELL_OV, "only OV latched (0x%lx)", (unsigned long)bms_monitor_get()->faults);
        if (lt - T_ON > worst) worst = lt - T_ON;
        if (lt - T_ON < best) best = lt - T_ON;
    }
    CHECK(worst <= BMS_FAULT_TIME_VOLTAGE_MS + 10, "worst OV reaction %u ms", worst);
    printf("OV reaction from onset: %u..%u ms (limit 500)\n", best, worst);

    /* 2. spikes shorter than the fault time never trip (they are seen by <= 3 samples) */
    for (uint32_t len = 50; len <= 250; len += 50) {
        reset_all(0);
        T_ON = 1005; T_OFF = 1005 + len;
        lt = simulate(0, 3000, cell_ov_window, ntc_ok);
        CHECK(lt == 0, "OV spike of %u ms tripped at %u", len, lt);
    }

    /* 3. regression: fault time must not collapse when the monitor runs in the same ms as the read
     *    (old code stored since = t|1, i.e. 1 ms in the future, and the unsigned difference wrapped) */
    reset_all(0);
    sample_cells(1000, 3700); bms_monitor_update(1000);
    sample_cells(1100, 4250); bms_monitor_update(1100);   /* same ms, even timestamp */
    CHECK(!bms_monitor_get()->ams_error, "immediate latch in the same ms as the sample");
    sample_cells(1200, 4250); bms_monitor_update(1200);
    CHECK(!bms_monitor_get()->ams_error, "latched after 100 ms");

    /* 4. under-voltage, over-temperature, open NTC */
    reset_all(0); T_ON = 1000; T_OFF = NEVER;
    lt = simulate(0, 3000, cell_uv_window, ntc_ok);
    CHECK(lt && lt - T_ON <= BMS_FAULT_TIME_VOLTAGE_MS + 10 && bms_monitor_get()->faults == BMS_FAULT_CELL_UV, "UV at %u", lt);
    worst = 0;
    for (uint32_t ph = 0; ph < 200; ph += 10) {
        reset_all(0); T_ON = 1000 + ph; T_OFF = NEVER;
        lt = simulate(0, 4000, cell_ok, ntc_ot_window);
        CHECK(lt && bms_monitor_get()->faults == BMS_FAULT_CELL_OT, "OT latched (phase %u, 0x%lx)", ph, (unsigned long)bms_monitor_get()->faults);
        if (lt - T_ON > worst) worst = lt - T_ON;
    }
    CHECK(worst < 1000, "worst OT reaction %u ms", worst);
    printf("OT reaction from onset: worst %u ms (limit 1000)\n", worst);
    reset_all(0); T_ON = 1000; T_OFF = NEVER;
    lt = simulate(0, 4000, cell_ok, ntc_open_window);
    CHECK(lt && lt - T_ON < 1000 && bms_monitor_get()->faults == BMS_FAULT_NTC, "open NTC (4760 mV) at %u faults 0x%lx", lt, (unsigned long)bms_monitor_get()->faults);

    /* 5. loss of data: cells stale after 400 ms, temperatures stale after 800 ms */
    reset_all(0);
    sample_cells(1000, 3700); sample_temps(1000, 2500);
    bms_monitor_update(1399);
    CHECK(!bms_monitor_get()->ams_error, "comm fault before 400 ms");
    bms_monitor_update(1400);
    CHECK(bms_monitor_get()->faults & BMS_FAULT_COMM, "comm fault at 400 ms");
    reset_all(0);
    for (uint32_t t = 1000; t <= 3000; t += 100) {       /* cells fresh, temperatures stop at 1000 */
        sample_cells(t, 3700); if (t == 1000) sample_temps(t, 2500);
        bms_monitor_update(t);
        if (t < 1800) CHECK(!bms_monitor_get()->ams_error, "temp stale fault too early (%u)", t);
    }
    CHECK(bms_monitor_get()->faults == BMS_FAULT_TEMP_COMM, "stale temperatures -> fault 0x%lx", (unsigned long)bms_monitor_get()->faults);

    /* 6. start-up grace */
    reset_all(5000);
    bms_monitor_update(5000 + BMS_STARTUP_GRACE_MS - 1);
    CHECK(!bms_monitor_get()->ams_error, "fault during the start-up grace");
    bms_monitor_update(5000 + BMS_STARTUP_GRACE_MS);
    CHECK(bms_monitor_get()->ams_error && stub_ams_pin, "no fault at the end of the start-up grace");

    /* 7. watchdog reset latches at once; AMS stays latched when values come back */
    reset_all(0);
    bms_monitor_force_fault(BMS_FAULT_WATCHDOG, 0);
    bms_monitor_update(0);
    CHECK(bms_monitor_get()->ams_error && stub_ams_pin && ams_error, "watchdog fault not latched");
    reset_all(0); T_ON = 1000; T_OFF = 1600;
    simulate(0, 4000, cell_ov_window, ntc_ok);
    CHECK(bms_monitor_get()->ams_error && stub_ams_pin && stub_err_led, "AMS released after the fault went away");
    CHECK(bms_monitor_get()->first_fault == BMS_FAULT_CELL_OV && bms_monitor_get()->first_fault_idx == 0, "first fault record");

    /* 8. HAL tick wrap-around (49.7 days) */
    base = 0xFFFFFFFFu - 1500u;
    reset_all(base);
    T_ON = base + 1000; T_OFF = NEVER;
    lt = simulate(base, base + 3000, cell_ov_window, ntc_ok);
    CHECK(lt && (uint32_t)(lt - T_ON) <= BMS_FAULT_TIME_VOLTAGE_MS + 10, "OV across the tick wrap at %u (+%u)", lt, lt - T_ON);
    reset_all(base);
    lt = simulate(base, base + 3000, cell_ok, ntc_ok);
    CHECK(lt == 0, "false fault across the tick wrap (faults 0x%lx)", (unsigned long)bms_monitor_get()->faults);

    /* 9. statistics */
    reset_all(0);
    sample_cells(100, 3650); sample_temps(100, 2500); bms_monitor_update(100);
    const bms_status_t *s = bms_monitor_get();
    CHECK(s->cell_min_mv == (raw_mv(3650) * 89 + 500) / 1000 && s->cell_min_idx == 0, "min cell %ld idx %u", (long)s->cell_min_mv, s->cell_min_idx);
    CHECK(s->cells_valid && s->temps_valid, "valid flags");
    CHECK(s->pack_mv > (int32_t)(3600 * BMS_CELLS_PER_SLAVE * BMS_N_SLAVES), "pack voltage %ld", (long)s->pack_mv);

    printf("%d/%d checks passed (N_SLAVES=%u)\n", checks - fails, checks, (unsigned)BMS_N_SLAVES);
    return fails ? 1 : 0;
}
