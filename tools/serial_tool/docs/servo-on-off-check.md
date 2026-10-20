# Servo On/Off Check

FEETECH STS3020のペン上げ/下げUART指令、現在位置、電源を確認するチェックです。
Waveshareなどの半二重TTL UART変換基板を使用してください。詳細な配線は[STS3020 Check](sts3020-check.md)。
ファームウェアコマンド名は`PENUP`と`PENDOWN`です。

## CSV

`tools/serial_tool/examples/servo_check.csv`

## Run

```bash
python tools/serial_tool/serial_send.py \
  --port /dev/cu.usbserial-023591AC \
  --csv tools/serial_tool/examples/servo_check.csv \
  --startup-delay 4 \
  --timeout 5 \
  --echo
```

## Checks

| Step | What to confirm |
|---|---|
| `CONFIG` | `STS3020 UART TX=32 RX=33`と表示される |
| `PEN_STATUS` | 通電してから位置が読める（サーボは動かない） |
| `POS` | 起動時は`PEN=UP` |
| `PENDOWN` | サーボがペン下げ方向へ動く |
| `PENUP` | サーボがペン上げ方向へ動く |
| repeated `PENDOWN`/`PENUP` | 引っかかりや過大角度がない |

## Related Config

- `PEN_SERVO_UART_TX_PIN` / `PEN_SERVO_UART_RX_PIN`
- `PEN_SERVO_ID` / `PEN_SERVO_UART_BAUD`
- `PEN_SERVO_SPEED` / `PEN_SERVO_ACCELERATION`
- `PEN_UP_ANGLE_DEG`
- `PEN_DOWN_ANGLE_DEG`

## Adjust

角度が合わない場合は`include/PlotterConfig.h`の角度を調整します。

```cpp
constexpr uint8_t PEN_UP_ANGLE_DEG = 30;
constexpr uint8_t PEN_DOWN_ANGLE_DEG = 70;
```

変更後は再ビルド・再書き込みしてください。

```bash
pio run
pio run --target upload
```

STS3020は外部7.4V電源（目安3A以上）を使用し、Core2からサーボには給電しません。初回はホーンを外してください。
