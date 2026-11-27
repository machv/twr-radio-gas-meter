# Gas usage measuring firmware for BigClown Core Module

This repository contains firmware for [Core Module](https://shop.bigclown.com/core-module) to measure gas usage via Elster IN-Z61 block.

## Home Assistant

With the MQTT integration already configured, the following setup lets you enter
the meter reading, send it explicitly, and see the response and listening countdown.
These examples use node `gas-meter:0`; change every topic if your node has a
different name.

The firmware in `src/application.c` subscribes to `usage/-/total/set` as an
integer counter. It stores that counter and publishes `usage/-/total` divided by
100. For example, a reading of `1359.12` must be sent as `135912`.

The device listens for commands for 60 seconds after startup or a physical button
press, and announces that window on `core/-/listening-timeout`.

### Helpers

Add the following to `configuration.yaml`. Merge entries into existing
`input_text:` and `timer:` sections instead of duplicating those keys.

```yaml
input_text:
  gas_meter_new_value:
    name: New gas meter reading
    max: 255

  gas_meter_expected_cents:
    name: Gas meter expected counter
    max: 20

  gas_meter_set_status:
    name: Gas meter set status
    max: 100
    initial: "Not sent"

timer:
  gas_meter_listening:
    name: Gas meter listening time
    duration: "00:01:00"
```

### Sending a reading

Add the following entry to `scripts.yaml`, assuming your configuration uses
`script: !include scripts.yaml`. If you define scripts directly in
`configuration.yaml`, nest this entry under `script:` instead. In the script
editor's YAML mode, omit the outer `set_gas_meter_value:` key.

The input accepts a dot as the decimal separator and up to six digits before it.
Missing decimal digits are padded and extra digits are discarded without
rounding: `1359`, `1359.1`, and `1359.129` send `135900`, `135910`, and `135912`,
respectively. The textbox remains unchanged.

```yaml
set_gas_meter_value:
  alias: Set gas meter reading
  mode: single
  sequence:
    - if: "{{ not (states('input_text.gas_meter_new_value') | trim | regex_match('^[0-9]{1,6}([.][0-9]*)?$')) }}"
      then:
        - stop: "Enter a non-negative reading with up to six digits before the decimal point."
          error: true

    - variables:
        whole: "{{ (states('input_text.gas_meter_new_value') | trim).split('.')[0] | int }}"
        fraction: >-
          {% set reading = states('input_text.gas_meter_new_value') | trim %}
          {{ ((reading.split('.')[1] if '.' in reading else '') ~ '00')[:2] | int }}

    - action: input_text.set_value
      target:
        entity_id: input_text.gas_meter_expected_cents
      data:
        value: "{{ (whole | int) * 100 + (fraction | int) }}"

    - action: input_text.set_value
      target:
        entity_id: input_text.gas_meter_set_status
      data:
        value: "Waiting for matching reading"

    - action: mqtt.publish
      data:
        topic: "node/gas-meter:0/usage/-/total/set"
        payload: "{{ (whole | int) * 100 + (fraction | int) }}"
        retain: false

    - wait_template: "{{ is_state('input_text.gas_meter_set_status', 'Matching reading received') }}"
      timeout: "00:00:30"
      continue_on_timeout: true

    - if: "{{ not wait.completed and is_state('input_text.gas_meter_set_status', 'Waiting for matching reading') }}"
      then:
        - action: input_text.set_value
          target:
            entity_id: input_text.gas_meter_set_status
          data:
            value: "No matching reading received"
```

String splitting is done on `states(...)` or a local Jinja variable, not on a
standalone script variable containing the raw reading. Home Assistant can
automatically convert numeric-looking script variables to numbers.

### Matching responses and starting the countdown

Append these automations to `automations.yaml`, assuming your configuration uses
`automation: !include automations.yaml`. If you define automations directly in
`configuration.yaml`, put these list entries under `automation:`. When creating
an automation through the editor's YAML mode, paste one entry without its leading
list dash.

```yaml
- id: gas_meter_set_confirmation
  alias: Gas meter matching reading
  triggers:
    - trigger: mqtt
      topic: "node/gas-meter:0/usage/-/total"
  conditions:
    - condition: state
      entity_id: input_text.gas_meter_set_status
      state: "Waiting for matching reading"
    - condition: template
      value_template: >-
        {{ is_number(trigger.payload)
           and is_number(states('input_text.gas_meter_expected_cents'))
           and ((trigger.payload | float * 100) | round(0) | int)
               == (states('input_text.gas_meter_expected_cents') | int) }}
  actions:
    - action: input_text.set_value
      target:
        entity_id: input_text.gas_meter_set_status
      data:
        value: "Matching reading received"

- id: gas_meter_listening_countdown
  alias: Gas meter listening countdown
  triggers:
    - trigger: mqtt
      topic: "node/gas-meter:0/core/-/listening-timeout"
  conditions:
    - condition: template
      value_template: >-
        {{ trigger.payload | regex_match('^[0-9]+$')
           and (trigger.payload | int) > 0 }}
  actions:
    - action: timer.start
      target:
        entity_id: timer.gas_meter_listening
      data:
        duration: "{{ trigger.payload | int }}"
```

The listener is already active before the script sends the command, so a fast
response can update the status even before the script reaches its wait step.
The response comparison rounds only to account for floating-point representation
of the reported value; the entered value is still truncated without rounding.

This is a matching-reading check, not a unique command acknowledgement: the
firmware uses the same total topic for periodic reports. A timeout means no
matching reading was observed within 30 seconds, not necessarily that the command
failed. MQTT publish success alone does not confirm device delivery.

The countdown restarts on every listening-timeout message, even if its payload
remains `60`. It estimates the remaining window from message arrival; transport
delays mean the actual window may end slightly earlier. Do not retain listening
announcements or command messages, as replaying them can produce a misleading
countdown or reset the meter.

### Dashboard

Add this as a manual dashboard card. It needs no custom cards. If you already have
a gas usage sensor, add its entity to the entities list. If Home Assistant assigned
a different script entity ID, adjust the button target accordingly.

```yaml
type: vertical-stack
cards:
  - type: entities
    title: Gas meter
    entities:
      - entity: sensor.gas_usage_komorany
      - entity: input_text.gas_meter_new_value
      - entity: script.set_gas_meter_reading
  - type: markdown
    content: '**Status:** {{ states(''input_text.gas_meter_set_status'') }}'
  - type: tile
    entity: timer.gas_meter_listening
    name: Time left to set meter
    tap_action:
      action: none

```

The status is read-only, and the tile shows the live countdown while active and
Idle afterward. 

### Applying and using the setup

Check the configuration and restart Home Assistant to load the new helpers,
scripts, and automations. For subsequent script or automation edits, reload them
after saving.

Enter the reading and press Enter to commit the textbox value. Press the device's
physical button, check that the countdown has started, and select Set before the
window expires.

If the input shows `unknown`, verify its entity ID in Developer Tools > States.
If a response is not matched, check the automation trace and the state of
`input_text.gas_meter_expected_cents`: sending `2567.63` should set that helper to
`256763`. Under Settings > Devices & services > MQTT, listen to the total topic
and verify that the bridge publishes a plain numeric payload on the configured
topic. The confirmation automation must be loaded and enabled.

## Development

### Init

```bash
git submodule update --init sdk

```

### Upgrade SDK version

```bash
git submodule update --remote --merge sdk
```
