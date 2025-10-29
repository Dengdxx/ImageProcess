// �Զ����ɵ� IPM ����� M (H_inv)
// ������ STM32 ����ʱ���� RAM ���ұ�

#include <stdint.h>

// ͼ��ߴ�궨��
#define IPM_OUT_HEIGHT 120
#define IPM_OUT_WIDTH  188
#define IPM_SRC_HEIGHT 120
#define IPM_SRC_WIDTH  188
#define IPM_INVALID_MARKER_UINT8 255

// �浥Ӧ�Ծ��� M = H_inverse (���� Flash)
// ���� STM32 �������и��������Ψһ����
const float ipm_matrix[3][3] = {
  { 0.70257766f, -0.20146909f, 23.97482162f },
  { -0.00000000f, 0.50390975f, 9.58992865f },
  { -0.00000000f, -0.00215475f, 0.95899286f },
};
