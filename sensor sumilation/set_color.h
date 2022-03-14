#define  T_BLACK   0x00
#define  T_DARK_BLUE       0x01
#define  T_DARK_GREEN 0x02
#define  T_DARK_CYAN       0x03
#define  T_DARK_RED     0x04
#define  T_DARK_VIOLET  0x05
#define  T_DARK_YELLOW   0x06
#define  T_GRAY 0x07
#define  T_LIGHT_GRAY      0x08
#define  T_BLUE     0x09
#define  T_GREEN   0x0a
#define  T_CYAN     0x0b
#define  T_RED      0x0c
#define  T_VIOLET  0x0d
#define  T_YELLOW 0x0e
#define  T_WHITE   0x0f
#define  T_INTENSITY     0x08     // 高輝度マスク
#define  T_RED_MASK     0x04     // 赤色ビット
#define  T_GREEN_MASK 0x02     // 緑色ビット
#define  T_BLUE_MASK   0x01     //  青色ビット



// SET(COLOR, 文字色 | 背景色)で色の設定ができるように#defineに文字を代入
#define  SET SetConsoleTextAttribute //SetConsoleTextAttributeをSETに代入
#define  COLOR hStdoutHandle //hStdoutHandleをCOLORに代入