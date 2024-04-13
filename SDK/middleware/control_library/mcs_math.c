/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
  * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
  * following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
  * disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  * following disclaimer in the documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
  * products derived from this software without specific prior written permission.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file      mcs_math.c
  * @author    MCU Algorithm Team
  * @brief     This file provides common math functions including trigonometric, coordinate transformation,
  *            square root math calculation.
  */

#include "mcs_math.h"
#include "mcs_math_const.h"
#include "mcs_assert.h"

/* Macro definitions --------------------------------------------------------------------------- */
#define SIN_TABLE \
    { \
        0, 51, 101, 151, 202, 252, 302, 352, 403, 453, 503, 553, 604, 654, 704, 754, 805, 855, 905, 955, 1006, 1056, \
        1106, 1156, 1207, 1257, 1307, 1357, 1407, 1458, 1508, 1558, 1608, 1659, 1709, 1759, 1809, 1859, 1909, \
        1960, 2010, 2060, 2110, 2160, 2210, 2261, 2311, 2361, 2411, 2461, 2511, 2561, 2611, 2662, 2712, 2762, \
        2812, 2862, 2912, 2962, 3012, 3062, 3112, 3162, 3212, 3262, 3312, 3362, 3412, 3462, 3512, 3562, 3612, \
        3662, 3712, 3762, 3812, 3862, 3912, 3962, 4012, 4061, 4111, 4161, 4211, 4261, 4311, 4360, 4410, 4460, \
        4510, 4560, 4609, 4659, 4709, 4759, 4808, 4858, 4908, 4958, 5007, 5057, 5107, 5156, 5206, 5255, 5305, \
        5355, 5404, 5454, 5503, 5553, 5602, 5652, 5701, 5751, 5800, 5850, 5899, 5949, 5998, 6048, 6097, 6146, \
        6196, 6245, 6294, 6344, 6393, 6442, 6492, 6541, 6590, 6639, 6689, 6738, 6787, 6836, 6885, 6934, 6983, \
        7033, 7082, 7131, 7180, 7229, 7278, 7327, 7376, 7425, 7474, 7523, 7572, 7620, 7669, 7718, 7767, 7816, \
        7865, 7913, 7962, 8011, 8060, 8108, 8157, 8206, 8254, 8303, 8352, 8400, 8449, 8497, 8546, 8594, 8643, \
        8691, 8740, 8788, 8837, 8885, 8933, 8982, 9030, 9078, 9127, 9175, 9223, 9271, 9320, 9368, 9416, 9464, \
        9512, 9560, 9608, 9656, 9704, 9752, 9800, 9848, 9896, 9944, 9992, 10040, 10088, 10136, 10183, 10231, \
        10279, 10327, 10374, 10422, 10470, 10517, 10565, 10612, 10660, 10707, 10755, 10802, 10850, 10897, 10945, \
        10992, 11039, 11087, 11134, 11181, 11228, 11276, 11323, 11370, 11417, 11464, 11511, 11558, 11605, 11652, \
        11699, 11746, 11793, 11840, 11887, 11934, 11981, 12027, 12074, 12121, 12167, 12214, 12261, 12307, 12354, \
        12400, 12447, 12493, 12540, 12586, 12633, 12679, 12725, 12772, 12818, 12864, 12910, 12957, 13003, 13049, \
        13095, 13141, 13187, 13233, 13279, 13325, 13371, 13417, 13463, 13508, 13554, 13600, 13646, 13691, 13737, \
        13783, 13828, 13874, 13919, 13965, 14010, 14056, 14101, 14146, 14192, 14237, 14282, 14327, 14373, 14418, \
        14463, 14508, 14553, 14598, 14643, 14688, 14733, 14778, 14823, 14867, 14912, 14957, 15002, 15046, 15091, \
        15136, 15180, 15225, 15269, 15314, 15358, 15402, 15447, 15491, 15535, 15580, 15624, 15668, 15712, 15756, \
        15800, 15844, 15888, 15932, 15976, 16020, 16064, 16108, 16151, 16195, 16239, 16282, 16326, 16369, 16413, \
        16456, 16500, 16543, 16587, 16630, 16673, 16717, 16760, 16803, 16846, 16889, 16932, 16975, 17018, 17061, \
        17104, 17147, 17190, 17233, 17275, 17318, 17361, 17403, 17446, 17488, 17531, 17573, 17616, 17658, 17700, \
        17743, 17785, 17827, 17869, 17911, 17953, 17995, 18037, 18079, 18121, 18163, 18205, 18247, 18288, 18330, \
        18372, 18413, 18455, 18496, 18538, 18579, 18621, 18662, 18703, 18745, 18786, 18827, 18868, 18909, 18950, \
        18991, 19032, 19073, 19114, 19155, 19195, 19236, 19277, 19317, 19358, 19398, 19439, 19479, 19520, 19560, \
        19600, 19641, 19681, 19721, 19761, 19801, 19841, 19881, 19921, 19961, 20001, 20041, 20080, 20120, 20160, \
        20199, 20239, 20278, 20318, 20357, 20397, 20436, 20475, 20514, 20554, 20593, 20632, 20671, 20710, 20749, \
        20788, 20826, 20865, 20904, 20943, 20981, 21020, 21058, 21097, 21135, 21174, 21212, 21250, 21289, 21327, \
        21365, 21403, 21441, 21479, 21517, 21555, 21593, 21630, 21668, 21706, 21744, 21781, 21819, 21856, 21894, \
        21931, 21968, 22005, 22043, 22080, 22117, 22154, 22191, 22228, 22265, 22302, 22339, 22375, 22412, 22449, \
        22485, 22522, 22558, 22595, 22631, 22667, 22704, 22740, 22776, 22812, 22848, 22884, 22920, 22956, 22992, \
        23028, 23063, 23099, 23135, 23170, 23206, 23241, 23277, 23312, 23347, 23383, 23418, 23453, 23488, 23523, \
        23558, 23593, 23628, 23662, 23697, 23732, 23767, 23801, 23836, 23870, 23904, 23939, 23973, 24007, 24042, \
        24076, 24110, 24144, 24178, 24212, 24245, 24279, 24313, 24347, 24380, 24414, 24447, 24481, 24514, 24547, \
        24581, 24614, 24647, 24680, 24713, 24746, 24779, 24812, 24845, 24878, 24910, 24943, 24975, 25008, 25040, \
        25073, 25105, 25137, 25170, 25202, 25234, 25266, 25298, 25330, 25362, 25393, 25425, 25457, 25488, 25520, \
        25551, 25583, 25614, 25646, 25677, 25708, 25739, 25770, 25801, 25832, 25863, 25894, 25925, 25955, 25986, \
        26017, 26047, 26078, 26108, 26138, 26169, 26199, 26229, 26259, 26289, 26319, 26349, 26379, 26409, 26438, \
        26468, 26498, 26527, 26557, 26586, 26616, 26645, 26674, 26703, 26732, 26761, 26790, 26819, 26848, 26877, \
        26906, 26934, 26963, 26991, 27020, 27048, 27077, 27105, 27133, 27161, 27189, 27217, 27245, 27273, 27301, \
        27329, 27356, 27384, 27412, 27439, 27467, 27494, 27521, 27549, 27576, 27603, 27630, 27657, 27684, 27711, \
        27737, 27764, 27791, 27817, 27844, 27870, 27897, 27923, 27949, 27976, 28002, 28028, 28054, 28080, 28106, \
        28132, 28157, 28183, 28209, 28234, 28260, 28285, 28310, 28336, 28361, 28386, 28411, 28436, 28461, 28486, \
        28511, 28535, 28560, 28585, 28609, 28634, 28658, 28682, 28707, 28731, 28755, 28779, 28803, 28827, 28851, \
        28875, 28898, 28922, 28946, 28969, 28993, 29016, 29039, 29063, 29086, 29109, 29132, 29155, 29178, 29201, \
        29223, 29246, 29269, 29291, 29314, 29336, 29359, 29381, 29403, 29425, 29447, 29469, 29491, 29513, 29535, \
        29557, 29578, 29600, 29622, 29643, 29664, 29686, 29707, 29728, 29749, 29770, 29791, 29812, 29833, 29854, \
        29874, 29895, 29916, 29936, 29956, 29977, 29997, 30017, 30037, 30057, 30077, 30097, 30117, 30137, 30157, \
        30176, 30196, 30215, 30235, 30254, 30273, 30292, 30312, 30331, 30350, 30369, 30387, 30406, 30425, 30443, \
        30462, 30481, 30499, 30517, 30536, 30554, 30572, 30590, 30608, 30626, 30644, 30661, 30679, 30697, 30714, \
        30732, 30749, 30767, 30784, 30801, 30818, 30835, 30852, 30869, 30886, 30903, 30919, 30936, 30952, 30969, \
        30985, 31002, 31018, 31034, 31050, 31066, 31082, 31098, 31114, 31129, 31145, 31161, 31176, 31192, 31207, \
        31222, 31237, 31253, 31268, 31283, 31298, 31312, 31327, 31342, 31357, 31371, 31386, 31400, 31414, 31429, \
        31443, 31457, 31471, 31485, 31499, 31513, 31526, 31540, 31554, 31567, 31581, 31594, 31607, 31620, 31634, \
        31647, 31660, 31673, 31685, 31698, 31711, 31724, 31736, 31749, 31761, 31773, 31786, 31798, 31810, 31822, \
        31834, 31846, 31857, 31869, 31881, 31892, 31904, 31915, 31927, 31938, 31949, 31960, 31971, 31982, 31993, \
        32004, 32015, 32025, 32036, 32047, 32057, 32067, 32078, 32088, 32098, 32108, 32118, 32128, 32138, 32148, \
        32157, 32167, 32177, 32186, 32195, 32205, 32214, 32223, 32232, 32241, 32250, 32259, 32268, 32276, 32285, \
        32294, 32302, 32311, 32319, 32327, 32335, 32343, 32351, 32359, 32367, 32375, 32383, 32390, 32398, 32405, \
        32413, 32420, 32427, 32435, 32442, 32449, 32456, 32463, 32469, 32476, 32483, 32489, 32496, 32502, 32509, \
        32515, 32521, 32527, 32533, 32539, 32545, 32551, 32557, 32562, 32568, 32573, 32579, 32584, 32589, 32595, \
        32600, 32605, 32610, 32615, 32619, 32624, 32629, 32633, 32638, 32642, 32647, 32651, 32655, 32659, 32663, \
        32667, 32671, 32675, 32679, 32682, 32686, 32689, 32693, 32696, 32700, 32703, 32706, 32709, 32712, 32715, \
        32718, 32720, 32723, 32726, 32728, 32730, 32733, 32735, 32737, 32739, 32741, 32743, 32745, 32747, 32749, \
        32751, 32752, 32754, 32755, 32756, 32758, 32759, 32760, 32761, 32762, 32763, 32764, 32764, 32765, 32766, \
        32766, 32767, 32767, 32767, 32767, 32767 \
    }

#define SIN_MASK 0x0C00
#define U0_90 0x0800
#define U90_180 0x0C00
#define U180_270 0x0000
#define U270_360 0x0400
#define SIN_TAB_LEN 0x03FF
#define Q15_BASE    32768
#define ANGLE_TO_INDEX_SHIFT  4

/* Private variables --------------------------------------------------------- */
const short g_sinTable[SIN_TAB_LEN + 1] = SIN_TABLE;

/**
  * @brief  Calculate sine and cosine function of the input angle.
  * @param  val: Output result, which contain the calculated sin, cos value.
  * @param  angle: The input parameter angle (Q15).
  * @retval None.
  */
void TrigCalc(TrigVal *val, signed short angle)
{
    MCS_ASSERT_PARAM(val != NULL);
    int indexS32;
    unsigned short indexU16;
    unsigned short angleRange;
    float sinVal = 0.0f;
    float cosVal = 0.0f;
    float oneDivPu = 0.00003051851f; /* 1.0f / 32767.0f */

    /* 16bit angle ->12bit index */
    indexS32 = Q15_BASE + (int)angle;
    indexU16 = (unsigned short)indexS32;
    indexU16 = indexU16 >> ANGLE_TO_INDEX_SHIFT;
    /* Calculate Angle Quadrant. */
    angleRange = indexU16 & SIN_MASK;
    indexU16 &= 0x03FF;

    /* Calculate symbols based on angle quadrants and look up tables. */
    switch (angleRange) {
        case U0_90:
            sinVal = g_sinTable[indexU16];
            cosVal = g_sinTable[SIN_TAB_LEN - indexU16]; /* indexU16 0 ~ 1023, indexU16 <= SIN_TAB_LEN */
            break;
        case U90_180:
            sinVal = g_sinTable[SIN_TAB_LEN - indexU16];
            cosVal = -g_sinTable[indexU16];
            break;
        case U180_270:
            sinVal = -g_sinTable[indexU16];
            cosVal = -g_sinTable[SIN_TAB_LEN - indexU16];
            break;
        case U270_360:
            sinVal = -g_sinTable[SIN_TAB_LEN - indexU16];
            cosVal = g_sinTable[indexU16];
            break;
        default:
            break;
    }
    /* Returns a sine-cosine result. */
    val->sin = sinVal * oneDivPu;
    val->cos = cosVal * oneDivPu;
}

/**
  * @brief  Park transformation: transforms stator values alpha and beta, which
  *         belong to a stationary albe reference frame, to a rotor flux
  *         synchronous reference dq frame.
  * @param  albe: Input alpha beta axis value.
  * @param  angle: Input the theta angle.
  * @param  dq: Output DQ axis value.
  * @retval None
  */
void ParkCalc(const AlbeAxis *albe, signed short angle, DqAxis *dq)
{
    MCS_ASSERT_PARAM(albe != NULL);
    MCS_ASSERT_PARAM(dq != NULL);
    float alpha = albe->alpha;
    float beta = albe->beta;
    TrigVal localTrigVal;
    /* The projection of ia, ib, and ic currents on alpha and beta axes is equivalent to that on d, q axes. */
    TrigCalc(&localTrigVal, angle);
    dq->d = alpha * localTrigVal.cos + beta * localTrigVal.sin;
    dq->q = -alpha * localTrigVal.sin + beta * localTrigVal.cos;
}

/**
  * @brief  Inverse Park transformation: transforms stator values d and q, which
  *         belong to a rotor flux synchronous reference dq frame, to a stationary
  *         albe reference frame.
  * @param  dq: Input DQ axis value.
  * @param  angle: Input the theta angle.
  * @param  albe: Output alpha beta axis value.
  * @retval None
  */
void InvParkCalc(const DqAxis *dq, signed short angle, AlbeAxis *albe)
{
    MCS_ASSERT_PARAM(dq != NULL);
    MCS_ASSERT_PARAM(albe != NULL);
    float d = dq->d;
    float q = dq->q;
    TrigVal localTrigVal;
    /* Inversely transform the d, q-axis current to alpha ,beta. */
    TrigCalc(&localTrigVal, angle);
    albe->alpha = d * localTrigVal.cos - q * localTrigVal.sin;
    albe->beta = d * localTrigVal.sin + q * localTrigVal.cos;
}

/**
  * @brief Clarke transformation: transforms stationary three-phase quantites to
  *        stationary albe quantites.
  * @param uvw: Clarke struct handle.
  * @param albe: AlbeAxis struct handle used to store the Clarke transform output.
  * @retval None.
  */
void ClarkeCalc(const UvwAxis *uvw, AlbeAxis *albe)
{
    MCS_ASSERT_PARAM(uvw != NULL);
    MCS_ASSERT_PARAM(albe != NULL);
    albe->alpha = uvw->u;
    albe->beta  = ONE_DIV_SQRT3 * (uvw->u + 2.0f * uvw->v);
}

/**
  * @brief This function returns the absolute value of the input value.
  * @param val: The quantity that wants to execute absolute operation.
  * @retval The absolute value of the input value.
  */
float Abs(float val)
{
    return (val >= 0.0f) ? val : (-val);
}

/**
  * @brief Clamp operation.
  * @param val Value that needs to be clamped.
  * @param upperLimit The upper limitation.
  * @param lowerLimit The lower limitation.
  * @retval Clamped value.
  */
float Clamp(float val, float upperLimit, float lowerLimit)
{
    MCS_ASSERT_PARAM(upperLimit > lowerLimit);
    float result;
    /* Clamping Calculation. */
    if (val >= upperLimit) {
        result = upperLimit;
    } else if (val <= lowerLimit) {
        result = lowerLimit;
    } else {
        result = val;
    }
    return result;
}

/**
  * @brief Get bigger value.
  * @param val1 The value to be compared.
  * @param val2 The value to be compared.
  * @retval The greater value.
  */
float Max(float val1, float val2)
{
    return ((val1 >= val2) ? val1 : val2);
}

/**
  * @brief Get smaller value.
  * @param val1 The value to be compared.
  * @param val2 The value to be compared.
  * @retval The smaller value.
  */
float Min(float val1, float val2)
{
    return ((val1 <= val2) ? val1 : val2);
}

/**
  * @brief Fast sqrt calculation using ASM.
  * @param val Float val.
  * @retval Sqrt result.
  */
float ASM_Sqrt(float val)
{
    MCS_ASSERT_PARAM(val >= 0.0f);
    float rd = val;

    __asm volatile("fsqrt.s %0, %1" : "=f"(rd) : "f"(val));

    return rd;
}
