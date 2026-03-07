/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cxx_include/esp_modem_api.hpp"
#include "cxx_include/esp_modem_dce_module.hpp"
#include "cxx_include/esp_modem_dte.hpp"

//  --- ESP-MODEM command module starts here ---
namespace esp_modem {

GenericModule::GenericModule(std::shared_ptr<DTE> dte, const dce_config *config) :
    dte(std::move(dte)),
    pdp(std::make_unique<PdpContext>(config->apn)),
    registration_timeout_ms(config->registration_timeout_ms) {}


#include "esp_modem_command_declare_helper.inc"

#define ESP_MODEM_DECLARE_DCE_COMMAND(name, type, ...) \
    type GenericModule::name(ESP_MODEM_COMMAND_PARAMS(__VA_ARGS__)) { \
        return esp_modem::dce_commands::name(dte.get() ESP_MODEM_COMMAND_FORWARD_AFTER(__VA_ARGS__)); }

#include "esp_modem_command_declare.inc"



// Usage examples:
// Zero arguments

// Helper to apply the correct macro to each parameter

//
// Repeat all declarations and forward to the AT commands defined in esp_modem::dce_commands:: namespace
//


#undef ESP_MODEM_DECLARE_DCE_COMMAND

//
// Handle specific commands for specific supported modems
//
command_result SIM7600::get_battery_status(int &voltage, int &bcs, int &bcl)
{
    return dce_commands::get_battery_status_sim7xxx(dte.get(), voltage, bcs, bcl);
}

command_result SIM7600::set_network_bands(const std::string &mode, const int *bands, int size)
{
    return dce_commands::set_network_bands_sim76xx(dte.get(), mode, bands, size);
}

command_result SIM7600::set_gnss_power_mode(int mode)
{
    return dce_commands::set_gnss_power_mode_sim76xx(dte.get(), mode);
}

command_result SIM7600::power_down()
{
    return dce_commands::power_down_sim76xx(dte.get());
}

command_result SIM7070::power_down()
{
    return dce_commands::power_down_sim70xx(dte.get());
}

command_result SIM7070::set_data_mode()
{
    return dce_commands::set_data_mode_alt(dte.get());
}

command_result SIM7000::power_down()
{
    return dce_commands::power_down_sim70xx(dte.get());
}

command_result SIM800::power_down()
{
    return dce_commands::power_down_sim8xx(dte.get());
}

command_result BG96::set_pdp_context(esp_modem::PdpContext &pdp)
{
    return dce_commands::set_pdp_context(dte.get(), pdp, 300);
}

// The GM02S(P) boots into CFUN=0 (minimum functionality), unlike most modems which default to
// CFUN=1, leaving the SIM interface disabled. CFUN=4 (airplane mode) is the minimum required
// state for SIM commands such as AT+CPIN to be available. For this reason, all SIM-related
// command overrides must call sqngm02s_prepare_sim() first.
static command_result sqngm02s_prepare_sim(SQNGM02S *m)
{
    int state;
    if (m->get_radio_state(state) != command_result::OK) {
        return command_result::FAIL;
    }
    if (state == 0) {
        return m->set_radio_state(4);
    }
    return command_result::OK;
}
command_result SQNGM02S::read_pin(bool &pin_ok)
{
    if (sqngm02s_prepare_sim(this) != command_result::OK) {
        return command_result::FAIL;
    }
    return GenericModule::read_pin(pin_ok);
}
command_result SQNGM02S::set_pin(const std::string &pin)
{
    if (sqngm02s_prepare_sim(this) != command_result::OK) {
        return command_result::FAIL;
    }
    return GenericModule::set_pin(pin);
}
bool SQNGM02S::setup_data_mode()
{
    if (set_echo(false) != command_result::OK) {
        return false;
    }

    int radio_state;
    if (get_radio_state(radio_state) == command_result::OK &&
            (radio_state == 1)) {
        return true;
    }

    if (set_pdp_context(*pdp) != command_result::OK) {
        return false;
    }
    if (set_radio_state(1) != command_result::OK) {
        return false;
    }
    constexpr int retry_delay_ms = 2000;
    const int max_retries = registration_timeout_ms / retry_delay_ms;
    int reg_state;
    for (int retry = 0; retry < max_retries; retry++) {
        if (get_network_registration_state(reg_state) == command_result::OK &&
                (reg_state == 1 || reg_state == 5)) {
            return true;
        }
        Task::Delay(retry_delay_ms);
    }
    return false;
}
}
