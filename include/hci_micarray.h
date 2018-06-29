/* -*-     c-basic-offset:4;     indent-tabs-mode: nil     -*- */
/* ==============================================================
 * Copyright (c) 1999-2016 SinoVoice Inc.  All rights reserved.
 */
/**
 * @file	hci_micarray.h
 * @brief	header file for hci micarray sdk.
 */

#ifndef HCI_MICARRAY_H
#define HCI_MICARRAY_H

#include "hci_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup HCI_SMA 灵云麦克风阵列SDK */
/* @{ */

#if defined(_WIN32) || defined(WIN32) || defined(WIN64)
#define MicArrayAPI _stdcall
#define MicArrayAPIExport
#else
#define MicArrayAPI
#define MicArrayAPIExport __attribute ((visibility("default")))
#endif

/// 麦克风阵列操作句柄定义
typedef void * MicArrayHandle;

/// 麦克风阵列会话句柄定义
typedef void * MicArraySession;

/// 麦克风阵列会话参数值定义
typedef void * MicArrayParamValue;

/**
 * 返回错误码定义：遵从灵云SDK的sys模块统一定义
 */
#ifndef MicArrayErrCode
#define MicArrayErrCode HCI_ERR_CODE
#endif

/**
 * MIC阵列类型
 */
#define MICARRAY_MODE_LINE                0           	///< MIC线性阵列
#define MICARRAY_MODE_CIRCULAR            1           	///< MIC圆形阵列
#define ISPEAKVQE_MIC_ARRAY_FRONT         2             ///< MIC朝前阵列（不定位方向）

const static unsigned int MICARRAY_MODE_DEFAULT = 0;           ///< MIC阵列的缺省类型是线阵

/**
 * MIC通道数
 */
#define MICARRAY_MIC_CHANNELNUM_MIN           2         ///< 最少支持2个MIC通道
#define MICARRAY_MIC_CHANNELNUM_MAX           8         ///< 最多支持8个MIC通道

const static unsigned int MICARRAY_MIC_CHANNELNUM_DEFAULT = 4; ///< MIC缺省通道数是4个

/**
 * 线阵MIC间距范围（单位：米）
 */
#define MICARRAY_MICDISTANCE_MIN              0.0f      ///< 最小MIC间距是0.0m
#define MICARRAY_MICDISTANCE_MAX              0.5f      ///< 最大MIC间距是0.5m
const static float MICARRAY_MIC_DISTANCE_DEFAULT = 0.035f;  ///< 线阵MIC缺省间距是35mm

/**
 * 圆阵MIC半径范围（单位：米）
 */
#define MICARRAY_MICRADIUS_MIN					0.0f	///< 圆阵MIC半径最小是0.0m
#define MICARRAY_MICRADIUS_MAX					0.5f	///< 圆阵MIC半径最大是0.5m
const static float MICARRAY_MIC_RADIUS_DEFAULT = 0.124f;	///< 圆阵MIC缺省半径是124mm

/**
 * 参考信道通道数
 */
#define MICARRAY_REF_CHANNELNUM_MIN              0      ///< 最少支持0个REF信道
#define MICARRAY_REF_CHANNELNUM_MAX              2      ///< 最多支持2个REF信道
const static unsigned int MICARRAY_REF_CHANNELS_DEFAULT = 1;   ///< 参考缺省信道数是1个

/**
 * 音频格式类型
 */
typedef enum //MICARRAY_AUDIO_FORMAT
{
    MICARRAY_AUDIO_FORMAT_FMT8BITS = 0,
    MICARRAY_AUDIO_FORMAT_FMT16BITS,
    MICARRAY_AUDIO_FORMAT_FMT24BITS,
} MICARRAY_AUDIO_FORMAT_E;

/**
 * 音频采样率类型
 */
typedef enum
{
    MICARRAY_SAMPLE_RATE_8K = 8000,    ///< 8k pcm
    MICARRAY_SAMPLE_RATE_16K = 16000,   ///< 16k pcm
} MICARRAY_SAMPLE_RATE_E;
const static unsigned int MICARRAY_SAMPLE_RATE_DEFAULT = 16000;///< 音频缺省采样频率是16K

/**
 * 数字旁瓣消除的各通道加权权值
 */
#define MICARRAY_DBF_WEIGHT_MIN		        0.0f        ///< 最小权值
#define MICARRAY_DBF_WEIGHT_MAX	            1.0f        ///< 最大权值
const static float MICARRAY_DBF_WEIGHT_DEFAULT = MICARRAY_DBF_WEIGHT_MAX;///< DBF各通道缺省加权值是1.0f

/**
 * 语音回声消除采用的算法
 */
#define MICARRAY_AEC_ALGORITHM_FREQUENCY		0	///< 频域算法
#define MICARRAY_AEC_ALGORITHM_TIME_NLMS		1	///< 时域的NLMS算法
const static unsigned int MICARRAY_AEC_ALGORITHM_DEFAULT = 0;///< AEC缺省采用算法是频域算法

/**
 * 圆阵方向计算的算法
 */
#define MICARRAY_DOA_SINGLE_CANDIDATE			0	///< 圆阵方向计算中对消采用的是时域方法，通过各通道时延进行估计,这是最新的算法
#define MICARRAY_DOA_MULTI_CANDIDATE_MATCH		1	///< 圆阵方向计算中对消采用的是频域方法，这是老的计算方法（r2057)
#define MICARRAY_DOA_MULTI_CANDIDATE_MAX		2	///< 圆阵方向计算的两候选输出能量最大一个算法
#define MICARRAY_DOA_MULTI_SINGLE_MAX			3	///< 圆阵方向计算的两候选加上默认的单候选输出能量最大一个算法

const static int MICARRAY_DOA_ALGORITHM_DEFAULT = 3;///< 圆阵方向计算的算法缺省值为0

#define MICARRAY_WAKECHECK_INDEX_INVALID		(-1)

/**
 * 唤醒检测模式定义
 */
typedef enum
{
    MICARRAY_WAKECHECK_MODE_STOP = 0,			///< 停止唤醒检测
    MICARRAY_WAKECHECK_MODE_ONESHOT,			///< 一次性唤醒检测
    MICARRAY_WAKECHECK_MODE_CONTINUOUS,			///< 持续性唤醒检测
} MICARRAY_WAKECHECK_MODE_E;

/**
 * 唤醒状态定义
 */
typedef enum
{
    MICARRAY_WAKESTATUS_SLEEPING = 0,			///< 无需唤醒
    MICARRAY_WAKESTATUS_WAKECHECKING,			///< 唤醒检测
    MICARRAY_WAKESTATUS_WAKEDUP,				///< 已唤醒
} MICARRAY_WAKESTATUS_E;

/**
 * 事件类型定义
 */
typedef enum
{
    MICARRAY_EVENT_MICREF_DELAY_DISORDER = 0,	///<   0: 输入音频：Mic/Ref同步关系紊乱
    MICARRAY_EVENT_VAD_OUTPUT_STARTED = 1,		///<   1: 输出音频：检测到VAD前端点
    MICARRAY_EVENT_VAD_OUTPUT_STOPPED,			///<   2: 输出音频：检测到VAD末端点
    MICARRAY_EVENT_DEVICE_WAKEDUP,				///<   3: 设备唤醒，如果不关注唤醒词序号，可使用该事件
    MICARRAY_EVENT_DEVICE_SLEPT,				///<   4: 设备休眠
    MICARRAY_EVENT_AECMODE_ENABLED,				///<   5: AEC模式开启
    MICARRAY_EVENT_AECMODE_DISABLED,			///<   6: AEC模式关闭
    MICARRAY_EVENT_AGCMODE_ENABLED,				///<   7: AGC模式开启
    MICARRAY_EVENT_AGCMODE_DISABLED,			///<   8: AGC模式关闭
    MICARRAY_EVENT_DOAMODE_ENABLED,				///<   9: DOA模式开启
    MICARRAY_EVENT_DOAMODE_DISABLED,			///<  10: DOA模式关闭
    MICARRAY_EVENT_DBFMODE_ENABLED,				///<  11: DBF模式开启
    MICARRAY_EVENT_DBFMODE_DISABLED,			///<  12: DBF模式关闭
    MICARRAY_EVENT_DENMODE_ENABLED,				///<  13: DEN降噪开启
    MICARRAY_EVENT_DENMODE_DISABLED,			///<  14: DEN降噪关闭
} MICARRAY_EVENT_E;

/**
 * 会话参数类型
 */
typedef enum MICARRAY_PARAM
{
    MICARRAY_PARAM_LOG_LEVEL = 1,				///<  1: logLevel, 麦克风阵列SDK日志级别，[1, 4]，可读写
    MICARRAY_PARAM_LOG_DATAMASK = 2,			///<  2: logDataMask，麦克风阵列SDK音频数据调试掩码，非负整数，可读写

    MICARRAY_PARAM_WAKE_CHECK_MODE = 11,		///< 11: wakeCheckMode, 唤醒模式设置：0，不唤醒；1，一次性唤醒；2，持续性唤醒，可读写
    MICARRAY_PARAM_WAKE_STATUS,					///< 12: wakeStatus, 当前唤醒状态：0，无需唤醒；1，唤醒检测中；2，已唤醒。只读
    MICARRAY_PARAM_WAKE_CHECK_CHANNEL_INDEX,	///< 13: wakeCheckChannelIndex,唤醒检测通道索引号，取值范围[1, micchannels]，缺省值为2，即第二通道，可读写

    MICARRAY_PARAM_AEC_STATUS = 21,				///< 21: aecStatus, 语音回声消除状态：0，关闭AEC；1，开启AEC，可读写
    MICARRAY_PARAM_AGC_STATUS,					///< 22: agcStatus, 自动增益控制状态：0，关闭AGC；1，开启AGC，可读写（暂不支持）
    MICARRAY_PARAM_DOA_STATUS,					///< 23: doaStatus, 声源定向估算状态：0，关闭DOA；1，开启DOA，可读写
    MICARRAY_PARAM_DBF_STATUS,					///< 24: dbfStatus, 角度指向增强状态：0，关闭DBF；1，开启DBF，可读写
    MICARRAY_PARAM_DEN_STATUS,					///< 25: denStatus, 语音噪声消除状态：0，关闭Den；1，开启Den，可读写

    MICARRAY_PARAM_VQE_DIRECTION = 31,	        ///< 31: vqeDirection, 定向增强角度，线阵范围：[0, 180]；圆阵范围：[0, 360]，可读写
    MICARRAY_PARAM_CURRENT_DIRECTION,			///< 32: currentDirection, 当前语音定向角度，线阵角度范围：[0, 180]，圆阵角度范围：[0, 360]，只读
    MICARRAY_PARAM_DBF_ALGORITHM,				///< 33: dbfAlgorithm, 指向增强算法：1，DBF；2，SDB，可读写
    //MICARRAY_PARAM_DBFPREGAIN,				///< 34: DbfPreGainFactor, 多路语音增强前的预增益系数设置，数字类型，取值范围：[1, 32]，缺省值是：1

    MICARRAY_PARAM_CALLBACK_STATEUPGRADED = 91,	///< 91: 系统状态变更回调， hci_micarray_statechange_callback，可读写
    MICARRAY_PARAM_CALLBACK_WAKEUP,				///< 92: 唤醒回调， hci_micarray_wakeup_callback，可读写
    MICARRAY_PARAM_CALLBACK_WAKEDIRECTION,		///< 93: 唤醒角度回调， hci_micarray_wakedirection_callback，可读写
    MICARRAY_PARAM_CALLBACK_VOICEREADY,			///< 94: 音频数据就绪回调， hci_micarray_voiceready_callback，可读写
} MICARRAY_PARAM_E;

//////////////////////////////////////////////////////////////////////////
// 数据结构定义

/**
 * @brief	语音唤醒音结果
 */
typedef struct MICARRAY_WAKE_RESULT_T
{
	int nWakeIndex;		///< 唤醒词序号：如1表示第一个唤醒词
	int nScore;			///< 唤醒结果置信度，范围[0,100]
	void *pDataBuff;	///< 唤醒词语音数据
	int nDataSize;		///< 唤醒词语音长度
} MICARRAY_WAKE_RESULT;

// 状态变更回调：系统状态变更后，SDK会调用该回调函数通知用户新的系统状态
typedef HCI_ERR_CODE (* hci_micarray_statechange_func)(
    _OPT_ _IN_ void	*pUserContext,					// [in] 用户上下文参数
    _MUST_ _IN_ MICARRAY_EVENT_E state);			// [in] 系统状态

// 唤醒回调：在检测到唤醒词后，SDK会调用该回调函数通知用户已唤醒
typedef HCI_ERR_CODE (* hci_micarray_wakeup_func)(
    _OPT_ _IN_ void *pUserContext,					// [in] 用户上下文参数
    _MUST_ _IN_ MICARRAY_WAKE_RESULT *pWakeResult);	// [in] 唤醒结果

// 方向回调：在检测到唤醒词后，SDK会调用该回调函数通知用户声源方位
typedef HCI_ERR_CODE (* hci_micarray_wakedirection_func)(
    _OPT_ _IN_ void *pUserContext,					// [in] 用户上下文参数
    _MUST_ _IN_ int nDirection);					// [in] 唤醒时的方向[0, 360)

// 音频处理结果回调，将处理完的单路音频返回给应用层
typedef HCI_ERR_CODE (* hci_micarray_voiceready_func)(
    _OPT_ _IN_ void *pUserContext,					// [in] 用户上下文参数
    _MUST_ _IN_ short *pVoiceData,					// [in] 输出音频流
    _MUST_ _IN_ int nVoiceSampleCount);				// [in] 输出音频流中包含的采样点个数

/**
*	@brief	 初始化麦克风阵列模块操作句柄
*	@details 麦克风阵列模块初始化。传入的pHandle不能为NULL，否则返回 HCI_ERR_PARAM_INVALID ；
* 		pHandle指针保存的是初始化成功后的操作句柄，调用者需要管理和维护pHandle对象。
*
*	@param pszConfigFile:	麦克风阵列SDK配置文件路径
*	@param pHandle:			操作句柄地址
*	@return
* @n
*	<table>
*		<tr><td>@ref HCI_ERR_NONE</td><td>正常</td></tr>
*		<tr><td>@ref HCI_ERR_CONFIG_INVALID</td><td>配置存在错误</td></tr>
*		<tr><td>@ref HCI_ERR_PARAM_INVALID</td><td>参数错误</td></tr>
*		<tr><td>@ref HCI_ERR_OUT_OF_MEMORY</td><td>内存分配失败</td></tr>
*		<tr><td>@ref HCI_ERR_SYS_NOT_INIT</td><td>HCI_SYS未初始化</td></tr>
*		<tr><td>@ref HCI_ERR_CONFIG_UNSUPPORT</td><td>配置项不支持</td></tr>
*		<tr><td>@ref HCI_ERR_CAPKEY_NOT_FOUND</td><td>没有找到指定的能力</td></tr>
*	</table>
* @n@n
*
* @note
*  在麦克风阵列初始化时，通过配置文件初始化的配置参数信息将会继承到后面的会话中，除非在会话中对这些参数进行重新设置
* @n@n
*
*/
MicArrayAPIExport MicArrayErrCode MicArrayAPI hci_micarray_init(
    _MUST_ _IN_ const char *pszConfigFile,
    _MUST_ _OUT_ MicArrayHandle *pHandle
);

/**
* @brief	版本信息获取
* @details	获取SDK版本信息
*
* @param   pszVersion: 字符串数组，用于保存版本信息
* @return
* @n
*	<table>
*		<tr><td>@ref HCI_ERR_NONE</td><td>操作成功</td></tr>
*		<tr><td>@ref HCI_ERR_PARAM_INVALID</td><td>参数错误</td></tr>
*		<tr><td>@ref HCI_ERR_UNSUPPORT</td><td>暂不支持</td></tr>
*	</table>
* @n@n
*
*/
MicArrayAPIExport MicArrayErrCode MicArrayAPI hci_micarray_get_version(
    _MUST_ _IN_OUT_ char *pszVersion);

/**
* @brief	创建麦克风阵列会话
* @details 创建麦克风阵列会话
*
* @param handle:	模块操作句柄
* @param scf:		状态变更事件回调
* @param wuf:		唤醒通知事件回调
* @param wdf:		唤醒角度事件回调
* @param vrf:		音频数据就绪回调
* @param pUserContext:	用户上下文参数
* @param pSession: 会话句柄地址
* @return
* @n
*	<table>
*		<tr><td>@ref HCI_ERR_NONE</td><td>正常</td></tr>
*		<tr><td>@ref HCI_ERR_PARAM_INVALID</td><td>参数错误</td></tr>
*		<tr><td>@ref HCI_ERR_CONFIG_INVALID</td><td>配置串参数错误</td></tr>
*		<tr><td>@ref HCI_ERR_OUT_OF_MEMORY</td><td>内存分配失败</td></tr>
*	</table>
* @n@n
*
*/
MicArrayAPIExport MicArrayErrCode MicArrayAPI hci_micarray_session_start(
    _MUST_ _IN_ const MicArrayHandle handle,
    _MUST_ _IN_ hci_micarray_statechange_func scf,
    _MUST_ _IN_ hci_micarray_wakeup_func wuf,
    _MUST_ _IN_ hci_micarray_wakedirection_func wdf,
    _MUST_ _IN_ hci_micarray_voiceready_func vrf,
    _OPT_ _IN_ void *pUserContext,
    _MUST_ _OUT_ MicArraySession *pSession
);

/**
* @brief	音频处理接口
* @details	用户通过该接口向SDK传送待处理音频数据，该数据的处理是通过异步方式执行的
*
* @param      session: 会话句柄
* @param voiceMicData: MIC语音数据，通道数由配置定义，运行期不允许改变
* @param voiceRefData: 参考语音数据，通道数由配置定义，运行期不允许改变；如果没有，传NULL
* @param voiceSamples: 一个通道的采样点数
* @return
* @n
*	<table>
*		<tr><td>@ref HCI_ERR_NONE</td><td>操作成功</td></tr>
*		<tr><td>@ref HCI_ERR_PARAM_INVALID</td><td>参数错误</td></tr>
*		<tr><td>@ref HCI_ERR_UNSUPPORT</td><td>暂不支持</td></tr>
*	</table>
* @n@n
*
*/
MicArrayAPIExport MicArrayErrCode MicArrayAPI hci_micarray_session_process(
    _MUST_ _IN_ MicArraySession session,
    _MUST_ _IN_ const short * voiceMicData,
    _MUST_ _IN_ const short * voiceRefData,
    _MUST_ _IN_ const int voiceSamples);

/**
* @brief	会话参数获取
* @details	获取会话参数
*
* @param    session		会话句柄
* @param	paramKey	要获取的参数类型
* @param	paramValue	获取参数输出缓存
* @return
* @n
*	<table>
*		<tr><td>@ref HCI_ERR_NONE</td><td>操作成功</td></tr>
*		<tr><td>@ref HCI_ERR_PARAM_INVALID</td><td>参数错误</td></tr>
*		<tr><td>@ref HCI_ERR_UNSUPPORT</td><td>暂不支持</td></tr>
*	</table>
* @n@n
*
*/
MicArrayAPIExport MicArrayErrCode MicArrayAPI hci_micarray_session_getparam(
    _MUST_ _IN_ MicArraySession session,
    _MUST_ _IN_ MICARRAY_PARAM_E paramKey,
    _MUST_ _OUT_ MicArrayParamValue *paramValue);

/**
* @brief	会话参数设置
* @details	设置会话参数
*
* @param    session		会话句柄
* @param	paramKey	要获取的参数类型
* @param	paramValue	获取参数输出缓存
* @return
* @n
*	<table>
*		<tr><td>@ref HCI_ERR_NONE</td><td>操作成功</td></tr>
*		<tr><td>@ref HCI_ERR_PARAM_INVALID</td><td>参数错误</td></tr>
*		<tr><td>@ref HCI_ERR_UNSUPPORT</td><td>暂不支持</td></tr>
*	</table>
* @n@n
*
*/
MicArrayAPIExport MicArrayErrCode MicArrayAPI hci_micarray_session_setparam(
    _MUST_ _IN_ MicArraySession session,
    _MUST_ _IN_ MICARRAY_PARAM_E paramKey,
    _MUST_ _IN_ MicArrayParamValue paramValue);

/**
* @brief	会话参数显示
* @details	显示会话参数
*
* @param    session		会话句柄
* @return
* @n
*	<table>
*		<tr><td>@ref HCI_ERR_PARAM_INVALID</td><td>参数错误</td></tr>
*		<tr><td>@ref HCI_ERR_NONE</td><td>操作成功</td></tr>
*	</table>
* @n@n
*
*/
MicArrayAPIExport MicArrayErrCode MicArrayAPI hci_micarray_session_printparams(
    _MUST_ _IN_ MicArraySession session);

/**
* @brief   停止麦克风阵列会话
* @details 停止麦克风阵列会话
*
* @param pSession: 会话句柄
* @return
* @n
*	<table>
*		<tr><td>@ref HCI_ERR_PARAM_INVALID</td><td>参数错误</td></tr>
*		<tr><td>@ref HCI_ERR_NONE</td><td>操作成功</td></tr>
*	</table>
* @n@n
*
*/
MicArrayAPIExport MicArrayErrCode MicArrayAPI hci_micarray_session_stop(
    _MUST_ _IN_OUT_ MicArraySession *pSession
);

/**
* @brief	 释放麦克风阵列模块操作句柄
* @details 释放麦克风阵列模块操作句柄。传入的pHandle不能为NULL，否则返回 HCI_ERR_PARAM_INVALID
*
* @param engineHandle: 要释放的麦克风阵列模块句柄
* @return
* @n
*	<table>
*		<tr><td>@ref HCI_ERR_NONE</td><td>正常</td></tr>
*		<tr><td>@ref HCI_ERR_PARAM_INVALID</td><td>参数错误</td></tr>
*	</table>
* @n@n
*
*/
MicArrayAPIExport MicArrayErrCode MicArrayAPI hci_micarray_release(
    _MUST_ _IN_OUT_ MicArrayHandle *pHandle
);

/* @} */
//////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}; /* extern "C" */
#endif

#endif // HCI_MICARRAY_H
