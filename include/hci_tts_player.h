/** 
 * @file    hci_tts_player.h 
 * @brief   HCI_TTS_PLAYER SDK ͷ�ļ�  
 */  

#ifndef __HCI_TTS_PLAYER_HEADER__
#define __HCI_TTS_PLAYER_HEADER__

#include "hci_sys.h"

#ifdef __cplusplus
extern "C"
{
#endif


/** @defgroup HCI_TTS_PLAYER_API ����TTS������API */
/* @{ */

    /** @defgroup HCI_PLAYER_STRUCT �ṹ�� */
    /* @{ */
    
    /**
	*@brief	ģ������
	*/
    #define MODULE_NAME    "HCI_TTS_PLAYER"

/**
 * @brief	�������ص�ʱ��֪ͨ�¼�
 */
typedef enum 
{
	PLAYER_EVENT_BEGIN,				///< ��ʼ����
	PLAYER_EVENT_PAUSE,				///< ��ͣ����
	PLAYER_EVENT_RESUME,			///< �ָ�����
	PLAYER_EVENT_PROGRESS,			///< ���Ž���
	PLAYER_EVENT_BUFFERING,			///< ���Ż���,�ȴ��ϳ�����
	PLAYER_EVENT_END,				///< �������
	PLAYER_EVENT_ENGINE_ERROR,		///< �������
	PLAYER_EVENT_DEVICE_ERROR,		///< �豸����
	PLAYER_EVENT_MALLOC_ERROR,		///< ����ռ�ʧ��
	PLAYER_EVENT_BUFFERING_END      ///< �����������������
} 
PLAYER_EVENT;

/**
 * @brief	���Ż�״̬
 */
typedef enum
{
	PLAYER_STATE_NOT_INIT,			///< û�г�ʼ��
	PLAYER_STATE_IDLE,				///< ����״̬��������������
	PLAYER_STATE_PLAYING,			///< ���ڲ���״̬
	PLAYER_STATE_PAUSE,				///< ��ͣ����״̬
	PLAYER_STATE_ERROR,				///< ����״̬
} 
PLAYER_STATE;

/**
* @brief	������������
*/
typedef enum
{
	PLAYER_ERR_UNKNOWN = -1,					///< -1: δ֪���󣬲������

	PLAYER_ERR_NONE = 0,						///< 0: �ɹ�

	PLAYER_ERR_NOT_INIT,						///< 1: û�г�ʼ��
	PLAYER_ERR_ALREADY_INIT,					///< 2: �Ѿ���ʼ��
	PLAYER_ERR_OPEN_DEVICE,						///< 3: ���豸����
	PLAYER_ERR_ALREADY_BEGIN,					///< 4: �Ѿ���ʼ
	PLAYER_ERR_NOT_BEGIN,						///< 5: û�п�ʼ
	PLAYER_ERR_OUT_OF_MEMORY,					///< 6: ����ռ�ʧ��
	PLAYER_ERR_ENGINE_ERROR,					///< 7: ��ʼ�� TTS SDK ����
}
PLAYER_ERR_CODE;

/**  
* @brief	�������¼��仯�ص�
* @note
* PLAYER_EVENT_PROGRESS �¼������ڸûص��д�������ֻ���� Callback_PlayerEventProgressChange �д���
* PLAYER_EVENT_ENGINE_ERROR��PLAYER_EVENT_DEVICE_ERROR �¼������ڸûص��д�����ֻ���� Callback_PlayerEventPlayerError �д���
* @param	ePlayerEvent		�ص�ʱ��֪ͨ�¼�
* @param	pUsrParam			�û��Զ������
*/
typedef void (HCIAPI * Callback_PlayerEventStateChange)(
	_MUST_ _IN_ PLAYER_EVENT ePlayerEvent,
	_OPT_ _IN_ void * pUsrParam );

/**  
* @brief	���Ž��ȱ仯֪ͨ
* @note
* ֻ�ᴥ�� PLAYER_EVENT_PROGRESS �¼�
* @param	ePlayerEvent		�ص�ʱ��֪ͨ�¼�
* @param	nStart				���ν��ȱ仯ʱ��ʼ���ļ�λ��(��λ���ֽ�)
* @param	nStop				���ν��ȱ仯ʱ�������ļ�λ��(��λ���ֽ�)
* @param	pUsrParam			�û��Զ������
*/ 
typedef void (HCIAPI * Callback_PlayerEventProgressChange)(
	_MUST_ _IN_ PLAYER_EVENT ePlayerEvent,
	_MUST_ _IN_ int nStart,
	_MUST_ _IN_ int nStop,
	_OPT_ _IN_ void * pUsrParam );

/**  
* @brief	����״̬�Ļص�
* ֻ�ᴥ�� PLAYER_EVENT_ENGINE_ERROR �¼�
* @param	ePlayerEvent		�ص�ʱ��֪ͨ�¼�
* @param	eErrorCode			������
* @param	pUsrParam			�û��Զ������
*/ 
typedef void (HCIAPI * Callback_PlayerEventPlayerError)(
	_MUST_ _IN_ PLAYER_EVENT ePlayerEvent,
	_MUST_ _IN_ HCI_ERR_CODE eErrorCode,
	_OPT_ _IN_ void * pUsrParam );

/**  
 * @brief	audiosession�ص�
 * @note
 * ֻ��iOSƽ̨ʹ��
 * @param	pExtendParam	��չ��������ǰ��Ч
 * @param	pUsrParam		�û�����
 */ 
typedef void (HCIAPI * Callback_PlayerSetAudioSession)(
    _MUST_ _IN_ void * pExtendParam,
    _OPT_ _IN_ void * pUsrParam);

/**  
* @ingroup HCI_PLAYER_STRUCT
* @brief	�������ص������Ľṹ��
*/
typedef struct _PLAYER_CALLBACK_PARAM {
	Callback_PlayerEventStateChange pfnStateChange;         ///< ������״̬�ص�
	void * pvStateChangeUsrParam;                           ///< ������״̬�ص��Զ������
	Callback_PlayerEventProgressChange pfnProgressChange;   ///< ���������Ȼص�
	void * pvProgressChangeUsrParam;                        ///< ���������Ȼص��Զ������
	Callback_PlayerEventPlayerError pfnPlayerError;         ///< ����������ص�
	void * pvPlayerErrorUsrParam;                           ///< ����������ص��Զ������
} PLAYER_CALLBACK_PARAM;

/* @} */

/** @defgroup HCI_PLAYER_FUNC ���� */
/* @{ */

/**  
 * @brief	������SDK��ʼ��
 * @param	pszSdkConfig		��ʼ�������������ô� ��ϸ˵�� �μ� hci_tts_init
 * @param	pCallbackParam		�ص������ļ���
 * @return	
 * @n
 *	<table>
 *		<tr><td>@ref PLAYER_ERR_NONE</td><td>�����ɹ�</td></tr>
 *		<tr><td>@ref PLAYER_ERR_ALREADY_INIT</td><td>�Ѿ���ʼ��</td></tr>
 *		<tr><td>@ref PLAYER_ERR_ENGINE_ERROR</td><td>TTS SDK ��ʼ��ʧ�ܣ�ʧ�ܵ�ԭ���ͨ�� Callback_PlayerEventPlayerError �ص�����</td></tr>
 *		<tr><td>@ref PLAYER_ERR_OUT_OF_MEMORY</td><td>����ռ�ʧ��</td></tr>
 *	</table>
 *
 *  <b>IOS��������</b>
 *  @n
 *  ������"volumeChannel"����ѡֵ"media"��"system"��Ĭ��ֵΪ"system"�������������ֵΪ"media"ʱ��TTS��������ʼ����
 *  �����Ƿ��ڲ���״̬���������������ڵ���ý����������ֵΪ"system"ʱ���ǲ���״̬���������������ڵ�������������
 *  TTS����������ʱ�����ڵ���ý����������ע��������ģʽ��Ҫ�������н������������е��ð�ť�������ش򿪡�
 */ 
PLAYER_ERR_CODE HCIAPI hci_tts_player_init(
		_MUST_ _IN_ const char * pszSdkConfig,
		_MUST_ _IN_ PLAYER_CALLBACK_PARAM *pCallbackParam);

/**  
 * @brief	����audiosession�ص�
 * @param	pfnCallBack		audiosession�ص�
 * @param	pUsrParam		�û�����
 * @return	
 * @n
 *	<table>
 *		<tr><td>@ref PLAYER_ERR_NONE</td><td>�����ɹ�</td></tr>
 *		<tr><td>@ref PLAYER_ERR_NOT_INIT</td><td>player��δ��ʼ��</td></tr>
 *	</table>
 * @n
 * @note
 * �˽ӿ�ֻ��iOSƽ̨ʹ��
 * playerĬ���������������������ֲ������������ġ�������Ӧ����Ҫ�������������ŵ�ʱ����Ӧ�жϽ�����Ӧ�Ĵ���
 * ���Ե��ô˺���������Callback_PlayerSetAudioSession�ص����ڻص��н���audiosession�����á��������audiosession�����
 * ��ƻ��AudioToolbox�Ĺٷ��ĵ���������һ������audiosession��ʾ��
 @code
 *  void playerSetAudioSessionCallback(void * pExtendParam,void * pUsrParam)
 *	{
 *		//��ʼ��audiosession
 *		AudioSessionInitialize(NULL, NULL, MyAudioSessionInterruptionListener, pUsrParam);
 *		//����kAudioSessionProperty_AudioCategory
 *		UInt32 sessionCategory = kAudioSessionCategory_MediaPlayback;
 *		AudioSessionSetProperty ( kAudioSessionProperty_AudioCategory, sizeof (sessionCategory), &sessionCategory);
 *		//����ʹ������������
 *		UInt32 audioRouteOverride = kAudioSessionOverrideAudioRoute_Speaker;
 *		AudioSessionSetProperty(kAudioSessionOverrideAudioRoute_None, sizeof(audioRouteOverride), &audioRouteOverride);
 *	}
 @endcode
 */ 
PLAYER_ERR_CODE hci_tts_player_set_audio_session_callback(
    _MUST_ _IN_ Callback_PlayerSetAudioSession pfnCallBack ,
    _OPT_ _IN_ void * pUsrParam);

    
/**
 * @brief	�õ��������ĵ�ǰ״̬
 * @return	�������ĵ�ǰ״̬
 */ 
PLAYER_STATE HCIAPI hci_tts_player_get_state();

/**  
 * @brief	��������ǰ�Ƿ����ֹͣ
 * @details	������������ PLAYER_STATE_PLAYING ���� PLAYER_STATE_PAUSEʱ�����ܹ���ֹͣ
 *          �˺���ֻ��Ϊ�˷������Ƿ����ֹͣ��һ��С���ߺ���
 * @return	
 * @n
 *	<table>
 *		<tr><td>true</td><td>����ֹͣ������</td></tr>
 *		<tr><td>false</td><td>������ֹͣ������</td></tr>
 *	</table>
 */ 
bool HCIAPI hci_tts_player_can_stop();

/**  
 * @brief	����������������һ���ı�
 * @details
 * ���ô��а���ʶ�����
 * @note
 * ���������ṩ�� pcm8k8bit pcm8k16bit pcm16k8bit pcm16k16bit pcm11k8bit pcm11k16bit ��ʽ��֧�� 
 * ������ǲ�����֧�ֵĸ�ʽ������ Callback_PlayerEventStateChange �д������豸ʧ�ܵ��¼�
 * @param	pszText				Ҫ���ŵ��ı���UTF-8���룬��'\0'����
 * @param	pszConfig			������ص����ô�����ͨ��deviceIdָ�������豸�����������ϸ˵���μ� hci_tts_session_start ��Ҫ��Ӻϳɵ�������
 * @return	
 * @n
 *	<table>
 *		<tr><td>@ref PLAYER_ERR_NONE</td><td>�����ɹ�</td></tr>
 *		<tr><td>@ref PLAYER_ERR_NOT_INIT</td><td>û�г�ʼ��</td></tr>
 *		<tr><td>@ref PLAYER_ERR_ALREADY_BEGIN</td><td>�Ѿ���ʼ����</td></tr>
 *		<tr><td>@ref PLAYER_ERR_OUT_OF_MEMORY</td><td>����ռ�ʧ��</td></tr>
 *	</table>
 */ 
PLAYER_ERR_CODE HCIAPI hci_tts_player_start(
	_MUST_ _IN_ const char * pszText,
	_MUST_ _IN_ const char * pszConfig);

/**  
 * @brief	��ͣ������
 * @return	
 * @n
 *	<table>
 *		<tr><td>@ref PLAYER_ERR_NONE</td><td>�����ɹ�</td></tr>
 *		<tr><td>@ref PLAYER_ERR_NOT_INIT</td><td>û�г�ʼ��</td></tr>
 *		<tr><td>@ref PLAYER_ERR_NOT_BEGIN</td><td>û�п�ʼ����</td></tr>
 *	</table>
 */ 
PLAYER_ERR_CODE HCIAPI hci_tts_player_pause();

/**  
 * @brief	�ָ�����������
 * @return	
 * @n
 *	<table>
 *		<tr><td>@ref PLAYER_ERR_NONE</td><td>�����ɹ�</td></tr>
 *		<tr><td>@ref PLAYER_ERR_NOT_INIT</td><td>û�г�ʼ��</td></tr>
 *		<tr><td>@ref PLAYER_ERR_NOT_BEGIN</td><td>û�п�ʼ����</td></tr>
 *	</table>
 */ 
PLAYER_ERR_CODE HCIAPI hci_tts_player_resume();

/**  
 * @brief	ֹͣ������
 * @return	
 * @n
 *	<table>
 *		<tr><td>@ref PLAYER_ERR_NONE</td><td>�����ɹ�</td></tr>
 *		<tr><td>@ref PLAYER_ERR_NOT_INIT</td><td>û�г�ʼ��</td></tr>
 *		<tr><td>@ref PLAYER_ERR_NOT_BEGIN</td><td>û�п�ʼ����</td></tr>
 *	</table>
 */ 
PLAYER_ERR_CODE HCIAPI hci_tts_player_stop();

/**  
 * @brief	������SDK����ʼ��
 * @return	
 * @n
 *	<table>
 *		<tr><td>@ref PLAYER_ERR_NONE</td><td>�����ɹ�</td></tr>
 *		<tr><td>@ref PLAYER_ERR_NOT_INIT</td><td>û�г�ʼ��</td></tr>
 *	</table>
 */ 
PLAYER_ERR_CODE HCIAPI hci_tts_player_release();

/* @} */
/* @} */
//////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif
//////////////////////////////////////////////////////////////////////////
#endif // _hci_cloud_tts_player_api_header_
