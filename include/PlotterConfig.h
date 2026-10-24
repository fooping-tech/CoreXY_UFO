#pragma once

#include <stdint.h>

// ============================================================================
// Board / build mode
// ============================================================================
// 対象ボード。現状はM5Stack Core2固定。
#define BOARD_M5STACK_CORE2 1

// 1にするとモータ出力を行わず、SerialログだけでCoreXY計算を確認する。
// 実機を動かす場合は0。新しいmotion処理を試す最初の段階では1が安全。
#ifndef SIMULATION_MODE
#define SIMULATION_MODE 0
#endif

// ============================================================================
// Motion geometry / feed / acceleration
// ============================================================================
// 1mm移動するために必要なステップ数。
// 決め方: motor step数、microstep、pulley歯数、belt pitchから計算し、
// 実測で補正する。例: 80 steps/mmなら10mm移動で800step。
constexpr float STEPS_PER_MM = 80.0f;

// CoreXYではXY速度方向によってA/Bモータのstep周波数が変わる。
// 最悪条件は45度方向で、片側モータがXY空間のsqrt(2)倍の速度になる。
constexpr float COREXY_MAX_MOTOR_GAIN = 1.41421356237f;

// 最大feed算出時の安全率。理論上限をそのまま使わず余裕を残す。
constexpr float SPEED_SAFETY = 1.0f;//0.80f;

// 標準feedを最大feedの何割にするか。
// DEFAULT_FEEDとMAX_FEEDを分け、通常描画が常に限界速度にならないようにする。
constexpr float DEFAULT_FEED_RATIO = 1.0f;//0.80f;

// モータSTEP周波数の上限 [steps/s]。
// 大きすぎると脱調、ノイズ、ドライバ発熱が増える。
constexpr uint32_t MAX_MOTOR_SPEED_STEPS_S = 30000;

// XYコマンドで許可する最大送り速度 [mm/min]。
// CoreXY最悪条件を考慮し、片側モータstep周波数が
// MAX_MOTOR_SPEED_STEPS_Sを超えない範囲から安全率込みで導出する。
constexpr float MAX_FEED_MM_MIN =
    MAX_MOTOR_SPEED_STEPS_S * 60.0f /
    (STEPS_PER_MM * COREXY_MAX_MOTOR_GAIN) * SPEED_SAFETY;

// feed未指定時や初期化時に使う標準送り速度 [mm/min]。
// 最大feedから比率で導出し、標準速度と限界速度の意味を分ける。
constexpr float DEFAULT_FEED_MM_MIN = MAX_FEED_MM_MIN * DEFAULT_FEED_RATIO;

// FastAccelStepperの初期速度 [steps/s]。
// 主に単軸テストや初期化時の値。通常XYはfeedから速度を決める。
constexpr uint32_t DEFAULT_MOTOR_SPEED_STEPS_S = static_cast<uint32_t>(
    DEFAULT_FEED_MM_MIN * STEPS_PER_MM / 60.0f + 0.5f);

// TrapezoidPlannerで使うXY空間の加速度 [mm/s^2]。
// 決め方: 低めから上げ、脱調や振動が出ない値にする。
// ペンが紙に接触する描画では負荷が増えるため、実機で段階的に確認する。
constexpr float DEFAULT_ACCEL_MM_S2 = 1000.0f;

// モータ加速度 [steps/s^2]。
// CoreXY最悪条件を考慮し、XY空間加速度にsteps/mmとsqrt(2)を掛けて導出する。
constexpr uint32_t DEFAULT_MOTOR_ACCEL_STEPS_S2 = static_cast<uint32_t>(
    DEFAULT_ACCEL_MM_S2 * STEPS_PER_MM * COREXY_MAX_MOTOR_GAIN + 0.5f);

// 将来の加速度clamp用の上限 [mm/s^2]。
// 現状はDEFAULT_ACCEL_MM_S2と同じ値。
constexpr float MAX_ACCEL_MM_S2 = DEFAULT_ACCEL_MM_S2;

// JunctionPlannerのlook-aheadで使う許容コーナー偏差 [mm]。
// 小さいほど角で減速し、大きいほど角を速く通過する。実機で角の丸まりと脱調を確認する。
constexpr float JUNCTION_DEVIATION_MM = 0.05f;

// classic jerk相当の簡易上限 [mm/s]。0以下なら無効。
// junction deviationだけで角が速すぎる場合の安全側制限として残す。
constexpr float CLASSIC_JERK_LIMIT_MM_S = 80.0f;

// motionTaskが連続XYをPlannerQueueへまとめるために待つ最大時間 [ms]。
// 0にすると、その時点でCommandQueueに溜まっているXYだけをlook-ahead対象にする。
constexpr uint32_t LOOKAHEAD_BATCH_COLLECT_MS = 15;

static_assert(STEPS_PER_MM > 0.0f, "STEPS_PER_MM must be > 0");
static_assert(COREXY_MAX_MOTOR_GAIN >= 1.4142f,
              "COREXY_MAX_MOTOR_GAIN must account for CoreXY diagonal motor speed");
static_assert(SPEED_SAFETY > 0.0f && SPEED_SAFETY <= 1.0f,
              "SPEED_SAFETY must be in (0, 1]");
static_assert(DEFAULT_FEED_RATIO > 0.0f && DEFAULT_FEED_RATIO <= 1.0f,
              "DEFAULT_FEED_RATIO must be in (0, 1]");
static_assert(MAX_MOTOR_SPEED_STEPS_S > 0,
              "MAX_MOTOR_SPEED_STEPS_S must be > 0");
static_assert(DEFAULT_FEED_MM_MIN <= MAX_FEED_MM_MIN,
              "DEFAULT_FEED_MM_MIN must be <= MAX_FEED_MM_MIN");
static_assert(DEFAULT_MOTOR_SPEED_STEPS_S <= MAX_MOTOR_SPEED_STEPS_S,
              "DEFAULT_MOTOR_SPEED_STEPS_S must be <= MAX_MOTOR_SPEED_STEPS_S");
static_assert(
    MAX_FEED_MM_MIN * STEPS_PER_MM * COREXY_MAX_MOTOR_GAIN / 60.0f
        <= MAX_MOTOR_SPEED_STEPS_S,
    "MAX_FEED_MM_MIN exceeds MAX_MOTOR_SPEED_STEPS_S in CoreXY worst case");
static_assert(DEFAULT_ACCEL_MM_S2 > 0.0f, "DEFAULT_ACCEL_MM_S2 must be > 0");
static_assert(MAX_ACCEL_MM_S2 >= DEFAULT_ACCEL_MM_S2,
              "MAX_ACCEL_MM_S2 must be >= DEFAULT_ACCEL_MM_S2");
static_assert(DEFAULT_MOTOR_ACCEL_STEPS_S2 > 0,
              "DEFAULT_MOTOR_ACCEL_STEPS_S2 must be > 0");
static_assert(JUNCTION_DEVIATION_MM > 0.0f,
              "JUNCTION_DEVIATION_MM must be > 0");

// AB_TIMED診断コマンドで許可する最小duration [us]。
// 短すぎるtimed moveはFastAccelStepper queueや機械側の切り分けに向かないため拒否する。
constexpr uint32_t AB_TIMED_MIN_DURATION_US = 1000;

// soft limit [mm]。homing後の通常XY移動をこの範囲に制限する。
// 決め方: 実際にペン先が安全に動ける作業範囲から少し余裕を引いた値。
// 2026-10-11: 暫定でX 500 / Y 250mm。実機ストローク未確認。
// 2026-10-22: ユーザー実機確認によりX 390 / Y 200mm(ユーザー指定値、余裕込み)。
// WebUIのSOFT_LIMIT(tools/webui/static/app.js)とserver.pyのテキスト生成上限も合わせる。
constexpr float X_MIN_MM = 0.0f;
constexpr float X_MAX_MM = 390.0f;
constexpr float Y_MIN_MM = 0.0f;
constexpr float Y_MAX_MM = 200.0f;

// JOB_END時の退避位置 [mm]。
// 描画物からペンを離し、次ジョブの準備状態を見やすくする。
constexpr bool JOB_END_PARK_ENABLED = true;
constexpr float JOB_END_PARK_X_MM = 5.0f;
constexpr float JOB_END_PARK_Y_MM = Y_MAX_MM - 5.0f;
constexpr float JOB_END_PARK_FEED_MM_MIN = 1200.0f;
// HOME(G28/JOB_BEGIN自動homing含む)完了後にJOB_END退避位置へ移動するか。
// false: HOMEはX/Y原点(リミット位置)で終了し、Y+方向の退避移動をしない。
// HOME_X/HOME_Yは従来から退避しない。JOB_ENDの退避はJOB_END_PARK_ENABLEDで別管理。
constexpr bool HOMING_PARK_AFTER_HOME_ENABLED = false;

// JOB_BEGIN時にhomedでない場合、自動でHOMEを実行するか。
// trueにするとJOB_BEGINで実機が動くため、limit switchとE-stop確認後に有効化する。
constexpr bool JOB_BEGIN_AUTO_HOME = true;

static_assert(JOB_END_PARK_X_MM >= X_MIN_MM && JOB_END_PARK_X_MM <= X_MAX_MM,
              "JOB_END_PARK_X_MM must stay inside X soft limits");
static_assert(JOB_END_PARK_Y_MM >= Y_MIN_MM && JOB_END_PARK_Y_MM <= Y_MAX_MM,
              "JOB_END_PARK_Y_MM must stay inside Y soft limits");

// ============================================================================
// Serial / stepper electrical timing
// ============================================================================
// USB Serialの通信速度。serial_tool側と一致させる。
constexpr uint32_t SERIAL_BAUD = 115200;

// DIRピン変更後、STEPを出すまで待つ時間 [us]。
// 決め方: ドライバのDIR setup timeより十分長くする。TMC2209では200usは保守的。
constexpr uint32_t DIR_CHANGE_DELAY_US = 200;

// ============================================================================
// Pen servo
// ============================================================================
// FEETECH STS3020 half-duplex TTL bus servo (external UART adapter required).
// Factory defaults are normally ID 1 and 1 Mbps; verify on the real device.
constexpr uint8_t PEN_SERVO_ID = 1;
constexpr uint32_t PEN_SERVO_UART_BAUD = 1000000;
constexpr uint16_t PEN_SERVO_SPEED = 300;
constexpr uint8_t PEN_SERVO_ACCELERATION = 20;
// Stop after a bounded delay instead of allowing XY to start with an unknown pen position.
constexpr uint32_t PEN_SERVO_MOVE_TIMEOUT_MS = 2000;
constexpr uint16_t PEN_SERVO_POSITION_TOLERANCE = 35;  // ~3 degrees

constexpr uint16_t PEN_SERVO_MAX_POSITION = 4095;
constexpr unsigned PEN_SERVO_FULL_TURN_DEG = 360;

// UFO構成ではID 1 = Z軸ドラム、ID 2 = アームで、ペン用サーボは未割当。
// false: PENUP/PENDOWNはERROR、G-codeのM3/M5とジョブ開始/終了のpen upは
// サーボを動かさずスキップする(ID 1のドラムを誤って回さないため)。
constexpr bool PEN_SERVO_ENABLED = false;

// ============================================================================
// Serial bus servos (test mode)
// ============================================================================
// テストモードで操作するSTS3020のID。ID 1 = Z軸(紐とドラム)、ID 2 = アーム。
constexpr uint8_t STS_SERVO_ID_MIN = 1;
constexpr uint8_t STS_SERVO_ID_MAX = 2;
// 1回転 = 4096 counts。
constexpr int32_t STS_COUNTS_PER_TURN = 4096;
// テストモードのサーボ速度。ACC単位は100 counts/s^2。
// 2026-10-22: 800から2400 counts/s(約0.6 rev/s、211deg/s)へ(ユーザー要望)。
// STS3020の無負荷最大は7.4Vで約4500 counts/s。ACCは2400到達0.48s(約580 counts)。
constexpr uint16_t STS_TEST_SPEED_COUNTS_PER_S = 2400;
constexpr uint8_t STS_TEST_ACC = 50;
// 1回のSERVO_TURN上限 [turn]と、追跡位置の上限(サーボの多回転goal +/-30719内)。
constexpr float STS_TEST_MAX_TURNS = 3.0f;
constexpr int32_t STS_TEST_ABS_LIMIT_COUNTS = 28000;
// 到達判定、poll周期、timeout余裕。
constexpr int STS_TEST_TOLERANCE_COUNTS = 5;
constexpr uint32_t STS_TEST_POLL_INTERVAL_MS = 20;
constexpr uint32_t STS_TEST_TIMEOUT_MARGIN_MS = 2000;
// 折り返し補正はpoll間の移動が半回転未満の間だけ有効。読出し欠落がこの時間を超えたら
// 追跡を無効化して止める。2400 counts/s * 0.25s = 600 counts(半回転2048より十分小さい)。
constexpr uint32_t STS_TEST_MAX_READ_GAP_MS = 250;
// hold check: hold + torque ON後この時間で+/-許容を超えて動けば即torque OFF。
constexpr uint32_t STS_TEST_HOLD_CHECK_MS = 200;
constexpr int STS_TEST_HOLD_CHECK_TOLERANCE_COUNTS = 10;
// runaway guard: 逆方向またはdelta超過がこの量を超えたら即torque OFF。
constexpr int STS_TEST_RUNAWAY_MARGIN_COUNTS = 50;
static_assert(static_cast<uint32_t>(STS_TEST_SPEED_COUNTS_PER_S) * STS_TEST_MAX_READ_GAP_MS / 1000 <
                  static_cast<uint32_t>(STS_COUNTS_PER_TURN / 4),
              "servo must move less than a quarter turn within STS_TEST_MAX_READ_GAP_MS");
static_assert(STS_TEST_MAX_TURNS * STS_COUNTS_PER_TURN < STS_TEST_ABS_LIMIT_COUNTS,
              "STS_TEST_MAX_TURNS must fit inside STS_TEST_ABS_LIMIT_COUNTS");

// ============================================================================
// Test mode XY / motor moves (no homing)
// ============================================================================
// TEST_JOGの1回の最大移動量 [mm]と既定速度 [mm/min]。limit ONで即停止する。
constexpr float TEST_JOG_MAX_DISTANCE_MM = 50.0f;
constexpr float TEST_JOG_DEFAULT_FEED_MM_MIN = 600.0f;
constexpr float TEST_JOG_MAX_FEED_MM_MIN = 1200.0f;
static_assert(TEST_JOG_MAX_FEED_MM_MIN <= MAX_FEED_MM_MIN,
              "TEST_JOG_MAX_FEED_MM_MIN must be <= MAX_FEED_MM_MIN");

// ============================================================================
// Atom JoyStick (ESP-NOW)
// ============================================================================
// 簡易操作用。本番の指示はPCからUSBシリアル。
// 2026-10-24: TEST_MODE不要に変更(CoreXY_UFO_Game HANDOFF_FIRMWARE_JOG §3.0)。
// HOMEDのジョグはHOMEDを保ちソフトリミット手前で止まる。シリアルJOG中は無視する。
constexpr bool JOYSTICK_ENABLED = true;
// ESP-NOWのWi-Fiチャネルは両ファームウェア共通のJoystickProtocol.hで定義する。
// 受け付けるAtomのMAC。全0なら起動後に最初に受信したAtomへ固定する(CONFIGに表示)。
constexpr uint8_t JOY_ALLOWED_MAC[6] = {0, 0, 0, 0, 0, 0};
// この時間パケットが来なければリンク断として停止する [ms]。
constexpr uint32_t JOY_LINK_TIMEOUT_MS = 250;
// XYジョグ速度 [mm/min]。左スティック押し込みでSLOW→FAST→VERY FAST→ULTRA→SLOWと切替。
// 2026-10-22: SLOW/FASTを600/3000へ、VERY FAST=FASTx1.5、ULTRA=FASTx2.5を追加(ユーザー要望)。
constexpr float JOY_SLOW_FEED_MM_MIN = 600.0f;
constexpr float JOY_FAST_FEED_MM_MIN = 3000.0f;
constexpr float JOY_VERY_FAST_FEED_MM_MIN = JOY_FAST_FEED_MM_MIN * 1.5f;  // 4500
constexpr float JOY_ULTRA_FEED_MM_MIN = JOY_FAST_FEED_MM_MIN * 2.5f;      // 7500
// XYジョグの加減速 [mm/s^2]。減速停止距離 v^2/2a: FAST約2mm、VERY FAST約4.7mm、ULTRA約13mm。
constexpr float JOY_ACCEL_MM_S2 = 600.0f;
// スティックのデッドゾーン(フルスケール比)。XY用と、サーボ(右スティック)用。
// サーボ側は中央付近のわずかなズレでジョグ開始(200msのhold check)しないよう大きめにする。
constexpr float JOY_DEADZONE = 0.08f;
constexpr float JOY_SERVO_DEADZONE = 0.25f;
// サーボ(ID1 Z / ID2 アーム)のジョグ最大速度 [counts/s]。2400 ~= 0.6 rev/s。
constexpr int32_t JOY_SERVO_MAX_COUNTS_S = STS_TEST_SPEED_COUNTS_PER_S;
// 右スティック上 / 右でサーボが+方向(時計回り)に回るなら1。実機で合わせる。
constexpr int8_t JOY_Z_SIGN = 1;
constexpr int8_t JOY_ARM_SIGN = 1;
// motionTaskのジョグ更新周期と、全軸0がこの時間続いたらセッション終了 [ms]。
constexpr uint32_t JOY_SESSION_PERIOD_MS = 20;
constexpr uint32_t JOY_SESSION_IDLE_EXIT_MS = 300;
// サーボジョグのgoal先行時間 [ms]。速度 x 先行時間だけ先にgoalを置く。
constexpr uint32_t JOY_SERVO_LEAD_MS = 200;
static_assert(JOY_ULTRA_FEED_MM_MIN <= MAX_FEED_MM_MIN,
              "JOY_ULTRA_FEED_MM_MIN must stay within MAX_FEED_MM_MIN");
static_assert(JOY_SLOW_FEED_MM_MIN > 0.0f && JOY_SLOW_FEED_MM_MIN <= JOY_FAST_FEED_MM_MIN &&
                  JOY_FAST_FEED_MM_MIN <= JOY_VERY_FAST_FEED_MM_MIN &&
                  JOY_VERY_FAST_FEED_MM_MIN <= JOY_ULTRA_FEED_MM_MIN,
              "joystick feeds must increase SLOW <= FAST <= VERY FAST <= ULTRA");

// ============================================================================
// Serial JOG / STOP (CoreXY_UFO_Game HANDOFF_FIRMWARE_JOG)
// ============================================================================
// デッドマン: 最後のJOGからこの時間内に次のJOGが来なければ減速停止する [ms]。
// ホストは押している間50ms(20Hz)ごとに送る。
constexpr uint32_t SERIAL_JOG_TIMEOUT_MS = JOY_LINK_TIMEOUT_MS;
// シリアルJOGのXY加減速 [mm/s^2]。ジョイスティックと同じ。
constexpr float SERIAL_JOG_ACCEL_MM_S2 = 600.0f;
// ジョグ中のJOG_POS出力周期 [ms](20Hz)。0で出力しない。
constexpr uint32_t JOG_POS_INTERVAL_MS = 50;
// HOMED中のソフトリミット減速。外向き速度vを v*T + v^2/(2*a*ratio) <= d に抑える
// (d=境界までの距離、T=JOG_SOFT_LIMIT_LATENCY_S)。この曲線に沿う減速度はa*ratio以下。
// CoreXYは各モータが独立に加減速するため、軸の減速度は最悪で公称の1/sqrt(2)になる。
// 0.5はそれより小さく取った安全率。Tは指令更新周期20ms+位置推定とFastAccelStepperの遅れ。
// 2026-10-24: 前回指令速度xT を距離から引く方式は実機で0.21mm越えた(カーブが急すぎ)ため変更。
constexpr float JOG_SOFT_LIMIT_BRAKE_RATIO = 0.5f;
constexpr float JOG_SOFT_LIMIT_LATENCY_S = 0.100f;
// 境界の手前に取る余裕 [mm]。遅れのある最後の接近で越えないため。停止位置は境界の約0.3〜0.5mm手前。
constexpr float JOG_SOFT_LIMIT_STOP_MARGIN_MM = 0.5f;
static_assert(SERIAL_JOG_ACCEL_MM_S2 > 0.0f && SERIAL_JOG_ACCEL_MM_S2 <= MAX_ACCEL_MM_S2,
              "SERIAL_JOG_ACCEL_MM_S2 must be in (0, MAX_ACCEL_MM_S2]");
static_assert(JOG_SOFT_LIMIT_BRAKE_RATIO > 0.0f && JOG_SOFT_LIMIT_BRAKE_RATIO < 0.7071f,
              "soft limit braking must stay below the CoreXY worst-case axis deceleration");

// ペン上げ/下げのサーボ角度 [deg]。
// 決め方: 機構に合わせて、紙を擦らない上げ角と十分に接地する下げ角を実測する。
constexpr uint8_t PEN_UP_ANGLE_DEG = 60;
constexpr uint8_t PEN_DOWN_ANGLE_DEG = 80;

// ============================================================================
// Motor direction
// ============================================================================
// A/Bモータの回転方向反転。
// 決め方: direction-checkで+X/+Yが期待方向に動かなければ該当軸を反転する。
// CoreXY式自体は変えず、ここで物理配線/取り付け差を吸収する。
constexpr bool MOTOR_A_DIRECTION_INVERTED = false;
constexpr bool MOTOR_B_DIRECTION_INVERTED = false;

// 論理A/Bモータと物理STEP/DIRピン(=ドライバ)の対応を入れ替える。
// true: 論理A(X+Y)をMOTOR_B_STEP/DIR_PINへ、論理B(X-Y)をMOTOR_A_STEP/DIR_PINへ出力する。
// 決め方: +X命令は正しいが+Y命令で-Yへ動く(=homing Yがswitchから離れる)場合、
// A/Bモータの配線が入れ替わっている。片側だけのDIRECTION_INVERTEDでは直らない
// (片側反転はX/Yが入れ替わり、両側反転はXも逆になる)ため、この設定で吸収する。
// MOTOR_A/B_DIRECTION_INVERTEDは入れ替え後の論理A/Bに対して適用される。
constexpr bool MOTOR_AB_SWAPPED = false;

// ============================================================================
// Homing / limit switches
// ============================================================================
// homing機能の有効/無効。
// 実機ではtrue。初期bring-upでlimit配線が未完成ならfalseも検討する。
constexpr bool HOMING_ENABLED = true;

// 原点探索方向。-1は負方向、+1は正方向へ探す。
// 決め方: limit switchを置いた原点方向に合わせる。
constexpr int8_t HOMING_X_DIR = -1;
constexpr int8_t HOMING_Y_DIR = -1;

// fast seek速度 [mm/min]。最初にlimitへ向かう速度。
// 決め方: 確実に停止できる範囲で速くする。
constexpr float HOMING_SEEK_FEED_MM_MIN = 1200.0f;

// slow seek速度 [mm/min]。backoff後に再度limitへ当てる低速。
// 決め方: 原点再現性を優先して低くする。
constexpr float HOMING_SLOW_FEED_MM_MIN = 60.0f;

// 通常homing中にlimitへ当たった後、いったん逃げる距離 [mm]。
// 決め方: switchが確実にOFFへ戻る距離より大きくする。
constexpr float HOMING_BACKOFF_MM = 8.0f;

// homing開始時からlimitがONだった場合に逃げられる最大距離 [mm]。
// 決め方: 起動時にswitch上へ乗っている可能性を考え、通常backoffより大きくする。
constexpr float HOMING_START_BACKOFF_MM = 40.0f;

// homingで探索を許す最大移動距離 [mm]。
// 決め方: 実ストロークより大きく、異常時に無限移動しない値にする。
constexpr float HOMING_MAX_TRAVEL_X_MM = 550.0f;
constexpr float HOMING_MAX_TRAVEL_Y_MM = 300.0f;

// homing完了時にMachineStateへ設定する座標 [mm]。
// 原点switchを作業座標0にするなら0。
constexpr float HOMING_SET_X_MM = 0.0f;
constexpr float HOMING_SET_Y_MM = 0.0f;

// limit switchのdebounce時間 [ms]。
// 決め方: チャタリングが消える最小値。長すぎると検出が遅れる。
constexpr uint32_t HOMING_LIMIT_DEBOUNCE_MS = 30;

// homing中ではない通常移動で、原点から離れた位置なのにlimitがONになった時に、
// アラームへ入れるまでの継続時間 [ms]。
// debounce後も継続するlimit ONは衝突または配線異常として速やかに止める。
constexpr uint32_t HARD_LIMIT_UNEXPECTED_ALARM_MS = 80;

// homing完了直後の通常移動で、原点limitがONのまま離れる方向へ動き出した時に、
// limitがOFFへ戻るまで許す最大移動距離 [mm]。
// これを超えてもOFFにならない場合はswitchが戻っていない、配線が短絡している、
// または移動方向が想定と違う可能性があるためalarmにする。
constexpr float NORMAL_MOVE_LIMIT_RELEASE_MM = 8.0f;

// trueならhoming完了前の通常XY移動を拒否する。
// 実機安全のためtrue推奨。ZEROはhoming扱いにしない。
constexpr bool HOMING_REQUIRE_HOMED_FOR_XY_MOVE = true;

// limit switchの有効極性。
// true: GPIO LOWでON。外付けpull-up + switchでGNDへ落とす配線。
// false: GPIO HIGHでON。
constexpr bool LIMIT_ACTIVE_LOW = true;

// ============================================================================
// LCD test mode
// ============================================================================
// Core2下部の中央ボタン(BtnB)をこの時間押し続けるとテストモードへ入る/抜ける [ms]。
// 決め方: 通常操作で誤って入らない長さ。
constexpr uint32_t TEST_MODE_HOLD_MS = 1500;

// テストモードの単独モータ移動速度 [mm/min相当]。
// 単独モータ移動はXYで1/sqrt(2)倍の対角移動になる。方向を目視しやすい低速にする。
// 例: 600mm/min = 800 steps/s。
constexpr float TEST_MODE_MOTOR_FEED_MM_MIN = 600.0f;

// テストモードで選べる単独モータ移動量 [steps]。STEPボタンで順に切り替える。
// 80 steps = モータ1mm(XY対角約0.7mm)、400 = 5mm、1600 = 20mm。
constexpr int32_t TEST_MODE_MOTOR_STEP_OPTIONS[] = {80, 400, 1600};
constexpr uint8_t TEST_MODE_MOTOR_STEP_DEFAULT_INDEX = 1;

static_assert(TEST_MODE_MOTOR_FEED_MM_MIN > 0.0f &&
                  TEST_MODE_MOTOR_FEED_MM_MIN <= MAX_FEED_MM_MIN,
              "TEST_MODE_MOTOR_FEED_MM_MIN must be in (0, MAX_FEED_MM_MIN]");
static_assert(TEST_MODE_MOTOR_STEP_DEFAULT_INDEX <
                  sizeof(TEST_MODE_MOTOR_STEP_OPTIONS) /
                      sizeof(TEST_MODE_MOTOR_STEP_OPTIONS[0]),
              "TEST_MODE_MOTOR_STEP_DEFAULT_INDEX out of range");

// ============================================================================
// NeoPixel status LEDs
// ============================================================================
// LED個数。実装しているNeoPixelの数に合わせる。
constexpr uint16_t NEOPIXEL_LED_COUNT = 8;

// 最大輝度。電源容量と眩しさを考えて低めに制限する。
constexpr uint8_t NEOPIXEL_BRIGHTNESS_MAX = 64;

// 起動時や通常設定で使う輝度。
constexpr uint8_t NEOPIXEL_BRIGHTNESS_DEFAULT = 24;

// LEDアニメーション更新周期 [ms]。小さいほど滑らかだがCPU負荷が増える。
constexpr uint32_t NEOPIXEL_FRAME_INTERVAL_MS = 33;

// 起動時のLED pattern。0はOFF。
constexpr uint8_t NEOPIXEL_INITIAL_PATTERN = 0;  // OFF

// ============================================================================
// TMC2209 normal profile
// ============================================================================
// 通常動作時のmicrostep。
// 決め方: 分解能、最大STEP周波数、静粛性のバランスで決める。
constexpr uint16_t TMC_NORMAL_MICROSTEPS = 16;

// 通常動作時のモータ電流 [mA RMS]。
// 決め方: 脱調しない最小値から始め、モータ/ドライバ温度を見て調整する。
constexpr uint16_t TMC_NORMAL_RMS_CURRENT_MA = 1100;

// trueならspreadCycle、falseならstealthChop寄りの設定。
// plotterの確実な駆動と高めの速度ではspreadCycle推奨。
constexpr bool TMC_NORMAL_SPREADCYCLE = true;

// TMCモジュール上のsense resistor値 [ohm]。
// 決め方: 使用モジュールの回路図/実装値に合わせる。
constexpr float TMC_R_SENSE_OHM = 0.11f;

// hold current比率。run currentに対する保持電流の割合。
// 小さいほど発熱は減るが、停止中に位置保持力が落ちる。
constexpr float TMC_HOLD_MULTIPLIER = 0.5f;

// TMCの電流senseレンジ設定。通常はモジュールと電流値に合わせる。
// falseのまま問題なければ変更しない。
constexpr bool TMC_CURRENT_VSENSE = false;

// TMC IHOLDDELAY。run電流からhold電流へ落とす遅延。
constexpr uint8_t TMC_IHOLDDELAY = 1;

// TMC TPOWERDOWN。停止後にpower downへ移るまでの時間設定。
constexpr uint8_t TMC_TPOWERDOWN = 20;

// spreadCycle chopper設定。基本はTMC2209の推奨値から始める。
// 異音、発熱、脱調がある場合だけ調整する。
constexpr uint8_t TMC_TOFF = 5;
constexpr uint8_t TMC_HSTRT = 5;
constexpr uint8_t TMC_HEND = 0;
constexpr uint8_t TMC_TBL = 2;

// StallGuardしきい値。現状は診断用placeholder寄り。
// 値が小さい/大きいと検出感度が変わるため、実機で調整する。
constexpr uint8_t TMC_SGTHRS_DEFAULT = 80;

// 移動直前のTMC2209ヘルスチェックの最小間隔 [ms]。周期チェックはしない。
// 前回チェックからこの時間が経ち、ステッパ停止中の移動コマンドでだけ実行する。
// GSTAT.reset(VM再投入等)を検出したら設定を再送する。モーション中はUARTを読まない。
constexpr uint32_t TMC_HEALTH_CHECK_INTERVAL_MS = 500;

// StallGuard/CoolStepの有効速度域しきい値。現状は広めに設定。
constexpr uint32_t TMC_TCOOLTHRS_DEFAULT = 0xFFFFF;

// ============================================================================
// Diagnostic motor melody
// ============================================================================
// 診断メロディ機能の有効/無効。
// TMC UARTとAモータ出力を確認するための機能。
constexpr bool MOTOR_MELODY_ENABLED = true;

// メロディ再生時だけ使うmicrostep。
// 音程を出しやすくするため通常動作とは別設定にする。
constexpr uint16_t MOTOR_MELODY_MICROSTEPS = 2;

// メロディ再生時の電流 [mA RMS]。
// 短時間動作だが、発熱と音量を見て調整する。
constexpr uint16_t MOTOR_MELODY_RMS_CURRENT_MA = 1200;

// メロディ再生時のchop mode。
constexpr bool MOTOR_MELODY_SPREADCYCLE = true;

// 音符間の隙間 [ms]。
constexpr uint16_t MOTOR_MELODY_NOTE_GAP_MS = 25;

// JOB_END時の終了ジングル。
// 既存曲そのものではなく、短い8-bit風のオリジナル和音パターンを鳴らす。
constexpr bool JOB_END_JINGLE_ENABLED = true;
