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
#define  T_INTENSITY     0x08     // ���P�x�}�X�N
#define  T_RED_MASK     0x04     // �ԐF�r�b�g
#define  T_GREEN_MASK 0x02     // �ΐF�r�b�g
#define  T_BLUE_MASK   0x01     //  �F�r�b�g



// SET(COLOR, �����F | �w�i�F)�ŐF�̐ݒ肪�ł���悤��#define�ɕ�������
#define  SET SetConsoleTextAttribute //SetConsoleTextAttribute��SET�ɑ��
#define  COLOR hStdoutHandle //hStdoutHandle��COLOR�ɑ��