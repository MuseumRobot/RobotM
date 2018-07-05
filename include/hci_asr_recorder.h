/**
 * @file    hci_asr_recorder.h
 * @brief   HCI_ASR_RECORDER SDK ͷ�ļ�
 */

#ifndef __HCI_ASR_RECORDER_HEADER__
#define __HCI_ASR_RECORDER_HEADER__

#include "hci_sys.h"
#include "hci_asr.h"

#ifdef __cplusplus
extern "C"
{
#endif
    
    /** @defgroup HCI_ASR_RECORDER_API ����ASR¼����API */
    /* @{ */
    
    /** @defgroup HCI_RECORDER_STRUCT �ṹ�� */
    /* @{ */
    
    /**
	*@brief	ģ������
	*/
    #define MODULE_NAME    "HCI_ASR_RECORDER"
    
    /**
     * @brief	¼�����ص�ʱ��֪ͨ�¼�
     */
    typedef enum
    {
        RECORDER_EVENT_BEGIN_RECORD,		///< ¼����ʼ
        RECORDER_EVENT_HAVING_VOICE,		///< �������� ��⵽ʼ�˵�ʱ��ᴥ�����¼�
        RECORDER_EVENT_NO_VOICE_INPUT,		///< û����������
        RECORDER_EVENT_BUFF_FULL,			///< ������������
        RECORDER_EVENT_END_RECORD,			///< ¼����ϣ��Զ����ֶ�������
        RECORDER_EVENT_BEGIN_RECOGNIZE,		///< ��ʼʶ��
        RECORDER_EVENT_RECOGNIZE_COMPLETE,	///< ʶ�����
        RECORDER_EVENT_ENGINE_ERROR,		///< �������
        RECORDER_EVENT_DEVICE_ERROR,		///< �豸����
        RECORDER_EVENT_MALLOC_ERROR,		///< ����ռ�ʧ��
        RECORDER_EVENT_INTERRUPTED,         ///< iOS�յ����������жϻᴥ�����¼���ͬʱֹͣ¼�����������߱��봦����¼�
        RECORDER_EVENT_PERMISSION_DENIED,   ///< iOSû����˷�ʹ��Ȩ��(ios7���ϰ汾���д��¼��������߱��봦����¼�)
        RECORDER_EVENT_TASK_FINISH,			///< ʶ���������
        RECORDER_EVENT_RECOGNIZE_PROCESS,	///< ʶ���м�״̬
    }
    RECORDER_EVENT;
    
    /**
     * @brief	¼����������
     */
    typedef enum
    {
        RECORDER_ERR_UNKNOWN = -1,					///< -1: δ֪���󣬲������
        
        RECORDER_ERR_NONE = 0,						///< 0: �ɹ�
        
        RECORDER_ERR_NOT_INIT,						///< 1: û�г�ʼ��
        RECORDER_ERR_ALREADY_INIT,					///< 2: �Ѿ���ʼ��
        RECORDER_ERR_ALREADY_BEGIN,					///< 3: �Ѿ���ʼ¼��
        RECORDER_ERR_NOT_BEGIN,						///< 4: û�п�ʼ¼��
        RECORDER_ERR_OUT_OF_MEMORY,					///< 5: ����ռ�ʧ��
        RECORDER_ERR_ENGINE_ERROR,					///< 6: �����������
		RECORDER_ERR_CONFIG_ITEM,					///< 7: ���ô�����
    }
    RECORDER_ERR_CODE;
    
    /**
     * @brief	¼�����¼��仯�ص�
     * @note
     * RECORDER_EVENT_RECOGNIZE_COMPLETE �¼������ڸûص��д�������ֻ���� Callback_RecorderEventRecogFinish �д���
     * RECORDER_EVENT_ENGINE_ERROR��RECORDER_EVENT_DEVICE_ERROR �¼������ڸûص��д�����ֻ���� Callback_RecorderErr �д���
     * @param	eRecorderEvent		�ص�ʱ��֪ͨ�¼�
     * @param	pUsrParam			�û��Զ������
     */
    typedef void (HCIAPI * Callback_RecorderEventStateChange)(
                                                              _MUST_ _IN_ RECORDER_EVENT eRecorderEvent,
                                                              _OPT_ _IN_ void * pUsrParam );
    
    /**
     * @brief	¼����Ƶ���ݻص�
     * @note
     * @param    pVoiceData      ��Ƶ����
     * @param    uiVoiceLen      ��Ƶ���ݳ���
     * @param    pUsrParam		�û��Զ������
     */
    typedef void (HCIAPI * Callback_RecorderRecording)(
                                                       _MUST_ _IN_ unsigned char * pVoiceData,
                                                       _MUST_ _IN_ unsigned int uiVoiceLen,
                                                       _OPT_ _IN_ void * pUsrParam
                                                       );
    
    /**
     * @brief	¼����ʶ����ɻص�
     * @note
     * ʶ��ɹ���ص�
     * @param	eRecorderEvent		ʶ������¼�
     * @param	pRecogResult		ʶ����
     * @param	pUsrParam			�û��Զ������
     */
    typedef void (HCIAPI * Callback_RecorderEventRecogFinish)(
                                                              _MUST_ _IN_ RECORDER_EVENT eRecorderEvent,
                                                              _MUST_ _IN_ ASR_RECOG_RESULT *pRecogResult,
                                                              _OPT_ _IN_ void * pUsrParam );
    
    
    /**
     * @brief	¼����ʶ���м����ص�
     * @note
     * ʶ���м����ص�
     * @param	eRecorderEvent		ʶ��������¼�
     * @param	pRecogResult		ʶ����
     * @param	pUsrParam			�û��Զ������
     */
    typedef void (HCIAPI * Callback_RecorderEventRecogProcess)(
                                                               _MUST_ _IN_ RECORDER_EVENT eRecorderEvent,
                                                               _MUST_ _IN_ ASR_RECOG_RESULT *pRecogResult,
                                                               _OPT_ _IN_ void * pUsrParam );
    
    
    /**
     * @brief	ASR SDK����ص�
     * @note
     * �����¼� RECORDER_EVENT_ENGINE_ERROR �� RECORDER_EVENT_DEVICE_ERROR ʱ�ص�
     * @param	eRecorderEvent		�ص�ʱ��֪ͨ�¼�
     * @param	nErrorCode			������
     * @param	pUsrParam			�û��Զ������
     */
    typedef void (HCIAPI * Callback_RecorderEventError)(
                                                        _MUST_ _IN_ RECORDER_EVENT eRecorderEvent,
                                                        _MUST_ _IN_ HCI_ERR_CODE eErrorCode,
                                                        _OPT_ _IN_ void * pUsrParam );
    
    /**
     * @brief	����audisession�ص�
     * @note
     * ֻ��iOSƽ̨ʹ��
     * @param	pExtendParam		��չ��������ǰ��Ч
     * @param	pUsrParam			�û�����
     */
    typedef bool (HCIAPI * Callback_RecorderSetAudioSession)(
                                                            _MUST_ _IN_ void * pExtendParam,
                                                            _OPT_ _IN_ void * pUsrParam);

    /**
     * @brief	¼�����ص������Ľṹ��
     */
    typedef struct _RECORDER_CALLBACK_PARAM {
        Callback_RecorderEventStateChange pfnStateChange;       ///< ¼����״̬�ص�
        void * pvStateChangeUsrParam;                           ///< ¼����״̬�ص��Զ������
        Callback_RecorderRecording pfnRecording;                ///< ¼�������ݻص�
        void * pvRecordingUsrParam;                             ///< ¼�������ݻص��Զ������
        Callback_RecorderEventRecogFinish pfnRecogFinish;       ///< ¼����ʶ������ص�
        void * pvRecogFinishUsrParam;                           ///< ¼����ʶ������ص��Զ������
        Callback_RecorderEventError pfnError;                   ///< ¼��������ص�
        void *pvErrorUsrParam;                                  ///< ¼��������ص��Զ������
        Callback_RecorderEventRecogProcess pfnRecogProcess;     ///< ¼����ʶ���м����ص�
        void *pvRecogProcessParam;                              ///< ¼����ʶ���м����ص��Զ������
    } RECORDER_CALLBACK_PARAM;

    /* @} */

    /** @defgroup HCI_RECOREDER_FUNC ���� */
    /* @{ */
    
    /**
     * @brief	¼����SDK��ʼ��
     * @param	pszAsrSdkConfig		¼������̨�����ô� �μ� hci_asr_init
     * @param	psCallbackParam		�ص������ļ���
     * @return
     * @n
     *	<table>
     *		<tr><td>@ref RECORDER_ERR_NONE</td><td>�����ɹ�</td></tr>
	 *		<tr><td>@ref RECORDER_ERR_CONFIG_ITEM</td><td>���ô�����</td></tr>
     *		<tr><td>@ref RECORDER_ERR_ENGINE_ERROR</td><td>ASR SDK ��ʼ��ʧ�ܣ�ʧ�ܵ�ԭ���ͨ�� Callback_RecorderErr �ص�����</td></tr>
     *		<tr><td>@ref RECORDER_ERR_ALREADY_INIT</td><td>�Ѿ���ʼ��</td></tr>
     *	</table>
	 * @par ���ô����壺
	 * ���ﶨ��¼���������ô������ã��������� �μ� hci_asr_init��
	 * @n@n
	 *	<table>
	 *		<tr>
	 *			<td><b>�ֶ�</b></td>
	 *			<td><b>ȡֵ��ʾ��</b></td>
	 *			<td><b>ȱʡֵ</b></td>
	 *			<td><b>����</b></td>
	 *			<td><b>��ϸ˵��</b></td>
	 *		</tr>
	 *		<tr>
	 *			<td>initCapKeys</td>
	 *			<td>�ַ������ο� hci_asr_init </td>
	 *			<td>��</td>
	 *			<td>׼�������б�</td>
	 *			<td>¼������֧���ƶ��﷨ʶ��</td>
	 *		</tr>
	 *	</table>
	 *
     */
    RECORDER_ERR_CODE HCIAPI hci_asr_recorder_init(
                                                   _MUST_ _IN_ const char * pszAsrSdkConfig,
                                                   _MUST_ _IN_ RECORDER_CALLBACK_PARAM *psCallbackParam);
    
    /**
     * @brief	����audiosession�ص�
     * @param	pfnCallBack		audiosession�ص�
     * @param	pUsrParam		�û�����
     * @return
     * @n
     *	<table>
     *		<tr><td>@ref RECORDER_ERR_NONE</td><td>�����ɹ�</td></tr>
     *		<tr><td>@ref RECORDER_ERR_NOT_INIT</td><td>recorder��δ��ʼ��</td></tr>
     *	</table>
     * @n
     * @note
     * �˽ӿ�ֻ��iOSƽ̨ʹ��
     * recorderĬ�ϲ������жϡ�������Ӧ����Ҫ��Ӧ�жϽ�����Ӧ�Ĵ���
     * ���Ե��ô˺���������Callback_RecorderSetAudioSession�ص����ڻص��н���audiosession�����á��������audiosession�����
     * ��ƻ��AudioToolBox�Ĺٷ��ĵ���������һ������audiosession��ʾ��
     @code
     *  void playerSetAudioSessionCallback(void * pExtendParam, void * pUsrParam)
     *	{
     *		//��ʼ��audiosession
     *		AudioSessionInitialize(NULL, NULL, MyAudioSessionInterruptionListener, pUsrParam);
     *		//����kAudioSessionProperty_AudioCategory
     *		UInt32 sessionCategory = kAudioSessionCategory_PlayAndRecord;
     *		AudioSessionSetProperty(kAudioSessionProperty_AudioCategory, sizeof(sessionCategory), &sessionCategory);
     *	}
     @endcode
     */
    RECORDER_ERR_CODE hci_asr_recorder_set_audio_session_callback(
                                                                  _MUST_ _IN_ Callback_RecorderSetAudioSession pfnCallBack ,
                                                                  _OPT_ _IN_ void * pUsrParam);
    
    /**
     * @brief	¼����SDK�����﷨
     * @param	pszAsrSdkConfig		¼������̨�����ô� �μ� hci_asr_load_grammar
     * @param	pszGrammarData		�﷨���ݻ����﷨�ļ�·��
     * @param   pnGrammarId         �﷨id
     * @return
     * @n
     *	<table>
     *		<tr><td>@ref RECORDER_ERR_NONE</td><td>�����ɹ�</td></tr>
     *		<tr><td>@ref RECORDER_ERR_ENGINE_ERROR</td><td>ASR SDK ��ʼ��ʧ�ܣ�ʧ�ܵ�ԭ���ͨ�� Callback_RecorderErr �ص�����</td></tr>
     *		<tr><td>@ref RECORDER_ERR_NOT_INIT</td><td>¼����û�г�ʼ��</td></tr>
     *	</table>
     */
    RECORDER_ERR_CODE HCIAPI hci_asr_recorder_load_grammar(
                                                           _OPT_ _IN_ const char * pszAsrSdkConfig,
                                                           _MUST_ _IN_ const char * pszGrammarData,
                                                           _MUST_ _OUT_ unsigned int * pnGrammarId
                                                           );
    
    /**
     * @brief	¼����SDKж���﷨
     * @param   nGrammarId         �﷨id
     * @return
     * @n
     *	<table>
     *		<tr><td>@ref RECORDER_ERR_NONE</td><td>�����ɹ�</td></tr>
     *		<tr><td>@ref RECORDER_ERR_ENGINE_ERROR</td><td>ASR SDK ��ʼ��ʧ�ܣ�ʧ�ܵ�ԭ���ͨ�� Callback_RecorderErr �ص�����</td></tr>
     *		<tr><td>@ref RECORDER_ERR_NOT_INIT</td><td>¼����û�г�ʼ��</td></tr>
     *	</table>
     */
    RECORDER_ERR_CODE HCIAPI hci_asr_recorder_unload_grammar(
                                                             _MUST_ _IN_ unsigned int nGrammarId
                                                             );
    
    /**  
     * @brief	¼����SDK����
     * @details
     * �����а����˵���������
     * @note
     * ¼�������ṩ�� pcm8k16bit pcm16k16bit ��ʽ��֧��
     * �������¼����֧�ֵĸ�ʽ������ Callback_RecorderEventStateChange �д������豸ʧ�ܵ��¼�
     * 
     * @param	pszConfig			���ô� ��ͨ��deviceIdָ��¼���豸����������μ� hci_asr_session_start
     * @param	pszGrammarData		�﷨�ļ�(ֻ�е����ô�grammarTypeΪwordlist��jsgf��wfstʱ��Ч��
     * @return	
     * @n
     *	<table>
     *		<tr><td>@ref RECORDER_ERR_NONE</td><td>�����ɹ�</td></tr>
     *		<tr><td>@ref RECORDER_ERR_NOT_INIT</td><td>û�г�ʼ��</td></tr>
	 *		<tr><td>@ref RECORDER_ERR_CONFIG_ITEM</td><td>���ô�����</td></tr>
     *		<tr><td>@ref RECORDER_ERR_ALREADY_BEGIN</td><td>�Ѿ���¼��</td></tr>
     *	</table>
	  * @par ���ô����壺
	 * ���ô�����"�ֶ�=ֵ"����ʽ������һ���ַ���������ֶ�֮����','�������ֶ������ִ�Сд��
	 * @n@n
	 * ����¼����������ã�
	 *	<table>
	 *		<tr>
	 *			<td><b>�ֶ�</b></td>
	 *			<td><b>ȡֵ��ʾ��</b></td>
	 *			<td><b>ȱʡֵ</b></td>
	 *			<td><b>����</b></td>
	 *			<td><b>��ϸ˵��</b></td>
	 *		</tr>
	 *		<tr>
	 *			<td>realtime</td>
	 *			<td>�ַ�������Чֵ{yes, rt}</td>
	 *			<td>yes</td>
	 *			<td>ʵʱʶ��ģʽ</td>
	 *			<td>yes,ʵʱʶ��ģʽ<br/>
	 *				rt,ʵʱ����ʶ����</td>
	 *		</tr>
	 *		<tr>
	 *			<td>continuous</td>
	 *			<td>�ַ�������Чֵ{yes, no}</td>
	 *			<td>yes</td>
	 *			<td>�Ƿ�����¼��</td>
	 *			<td>yes,����¼��<br/>
	 *				no,���¼������⵽һ�ζ˵�ĩβ��ֹͣ����Ҫ�ٴο��������ܽ���¼��</td>			
	 *		</tr>
	 *		<tr>
	 *			<td>audioformat</td>
	 *			<td>�ַ�������Чֵ{pcm16k16bit, pcm8k16bit}</td>
	 *			<td>pcm16k16bit</td>
	 *			<td>¼����Ƶ��ʽ</td>
	 *			<td>pcm16k16bit,16k16bit pcm¼��<br/>
	 *				pcm8k16bit,8k16bit pcm¼��</td>
	 *		</tr>
	 *	</table>
	 * @n@n
	 * ���⣬���ﻹ���Դ���ʶ����������ΪĬ��������μ� hci_asr_recog() ��
	 * ���pszConfig��û��asrʶ���capkey���������¼�����ܣ�ͨ�������ص��ӿ��������������ݡ�
     */ 
    RECORDER_ERR_CODE HCIAPI hci_asr_recorder_start(
                                                    _MUST_ _IN_ const char * pszConfig,
                                                    _OPT_ _IN_ const char * pszGrammarData);
    
    /**  
     * @brief	����¼������ʶ��
     * @details �˲�����ʾ����¼������ʶ�����ϣ������¼����ʶ������� hci_asr_recorder_stop_and_recog()
     * @return	
     * @n
     *	<table>
     *		<tr><td>@ref RECORDER_ERR_NONE</td><td>�����ɹ�</td></tr>
     *		<tr><td>@ref RECORDER_ERR_NOT_INIT</td><td>û�г�ʼ��</td></tr>
     *		<tr><td>@ref RECORDER_ERR_NOT_BEGIN</td><td>û�п�ʼ¼��</td></tr>
     *	</table>
     */ 
    RECORDER_ERR_CODE HCIAPI hci_asr_recorder_cancel();
    
    /**  
     * @brief	����¼��������ʶ��
     * @details �˲�����ʾ����¼��ͬʱ����ʶ����̣����ϣ������¼��ʱ��ȥʶ������� hci_asr_recorder_cancel()
     * @return	
     * @n
     *	<table>
     *		<tr><td>@ref RECORDER_ERR_NONE</td><td>�����ɹ�</td></tr>
     *		<tr><td>@ref RECORDER_ERR_NOT_INIT</td><td>û�г�ʼ��</td></tr>
     *		<tr><td>@ref RECORDER_ERR_NOT_BEGIN</td><td>û�п�ʼ¼��</td></tr>
     *	</table>
     */ 
    RECORDER_ERR_CODE HCIAPI hci_asr_recorder_stop_and_recog();
    
    /**  
     * @brief	�ϴ�ʶ���ȷ�Ͻ��
     * @return	
     * @n
     *	<table>
     *		<tr><td>@ref RECORDER_ERR_NONE</td><td>�����ɹ�</td></tr>
     *		<tr><td>@ref RECORDER_ERR_NOT_INIT</td><td>û�г�ʼ��</td></tr>
     *		<tr><td>@ref RECORDER_ERR_OUT_OF_MEMORY</td><td>�ڴ����ʧ��</td></tr>
     *	</table>
     * @note
     * iOS ƽ̨��֧�ִ˽ӿڣ����ú󲻽����κβ��������� RECORDER_ERR_NONE
     */ 
    RECORDER_ERR_CODE HCIAPI hci_asr_recorder_confirm(
                                                      _MUST_ _IN_ ASR_CONFIRM_ITEM * psAsrConfirmItem);
    
    /**  
     * @brief	ASR ¼��������ʼ��
     * @return	
     * @n
     *	<table>
     *		<tr><td>@ref RECORDER_ERR_NONE</td><td>�����ɹ�</td></tr>
     *		<tr><td>@ref RECORDER_ERR_NOT_INIT</td><td>û�г�ʼ��</td></tr>
     *	</table>
     */ 
    RECORDER_ERR_CODE HCIAPI hci_asr_recorder_release();
    
     /* @} */
    /* @} */
    //////////////////////////////////////////////////////////////////////////
    
#ifdef __cplusplus
}
#endif

#endif // _hci_cloud_asr_recorder_api_header_
