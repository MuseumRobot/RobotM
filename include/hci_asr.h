/** 
* @file    hci_asr.h 
* @brief   HCI_ASR SDK ͷ�ļ�  
*/  

#ifndef __HCI_ASR_HEADER__
#define __HCI_ASR_HEADER__

#include "hci_sys.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** @defgroup HCI_ASR ����ASR����API */
    /* @{ */
    //////////////////////////////////////////////////////////////////////////

    /** @defgroup HCI_ASR_STRUCT  �ṹ�� */
    /* @{ */
    //////////////////////////////////////////////////////////////////////////
    
    /**
	*@brief	ģ������
	*/
    #define ASR_MODULE    "HCI_ASR"
    
    /**
	*@brief	�ϴ���ASRȷ�Ͻ����Ϣ
	*/
	typedef struct _tag_ASR_CONFIRM_ITEM 
	{
		/// ȷ�ϵ�ʶ��������
		char * pszResult;		
	} ASR_CONFIRM_ITEM;

    /**
	*@brief	ASRʶ���ѡ�����Ŀ
	*/
	typedef struct _tag_ASR_RECOG_RESULT_ITEM 
	{
		/// ��ѡ�����ֵ, ��ֵԽ�ߣ�Խ����
		unsigned int		uiScore;

		/// ��ѡ����ַ�����UTF-8���룬��'\0'����
		char *				pszResult;
	} ASR_RECOG_RESULT_ITEM;

	/**
	*@brief	ASRʶ�����ķ��ؽ��
	*/
	typedef struct _tag_ASR_RECOG_RESULT 
	{
		/// ʶ���ѡ����б�
		ASR_RECOG_RESULT_ITEM *	psResultItemList;

		/// ʶ���ѡ�������Ŀ
		unsigned int		uiResultItemCount;
	} ASR_RECOG_RESULT;

    /* @} */


    /** @defgroup HCI_ASR_FUNC  ���� */
    /* @{ */
    //////////////////////////////////////////////////////////////////////////    

	/**  
	* @brief	����ASR����  ��ʼ��
	* @param	pszConfig	��ʼ�����ô���ASCII�ַ�������ΪNULL����'\0'����
	* @return
	* @n
	*	<table>
	*		<tr><td>@ref HCI_ERR_NONE</td><td>�����ɹ�</td></tr>
	*		<tr><td>@ref HCI_ERR_SYS_NOT_INIT</td><td>HCI SYS ��δ��ʼ��</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_ALREADY_INIT</td><td>�Ѿ���ʼ������</td></tr>
	*		<tr><td>@ref HCI_ERR_CONFIG_INVALID</td><td>���ò����������趨ֵ�Ƿ������ʽ�����</td></tr>
	*		<tr><td>@ref HCI_ERR_CONFIG_DATAPATH_MISSING</td><td>ȱ�ٱ����dataPath������</td></tr>
	*		<tr><td>@ref HCI_ERR_CONFIG_CAPKEY_NOT_MATCH</td><td>CAPKEY�뵱ǰ���治ƥ��</td></tr>
	*		<tr><td>@ref HCI_ERR_CAPKEY_NOT_FOUND</td><td>û���ҵ�ָ��������</td></tr>
	*		<tr><td>@ref HCI_ERR_LOAD_FUNCTION_FROM_DLL</td><td>���ؿ⺯��ʧ��</td></tr>
	*	</table>
	*
	* @par ���ô����壺
	* ���ô�����"�ֶ�=ֵ"����ʽ������һ���ַ���������ֶ�֮����','�������ֶ������ִ�Сд��
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
	*			<td>dataPath</td>
	*			<td>�ַ������磺./data/</td>
	*			<td>��</td>
	*			<td>����ʶ�𱾵���Դ����·��</td>
	*			<td>����·���´�ű�������������Դ</td>
	*		</tr>
	*		<tr>
	*			<td>initCapKeys</td>
	*			<td>�ַ������ο� @ref hci_asr_page </td>
	*			<td>��</td>
	*			<td>׼�������б�</td>
	*			<td>��Ҫ׼���������б����������';'����</td>
	*		</tr>
	*		<tr>
	*			<td>fileFlag</td>
	*			<td>�ַ�������Чֵ{none, android_so}</td>
	*			<td>none</td>
	*			<td>��ȡ�����ļ�����������</td>
	*			<td>�μ������ע��</td>
	*		</tr>
	*	</table>
	*
	*  @note
	*  <b>Android��������</b>
	*  @n
	*  ��fileFlagΪandroid_soʱ�����ر�����Դ�ļ�ʱ�Ὣ�����Ŀ��ļ�������Ϊso�ļ������м��ء�
	*  ���磬��ʹ�õĿ�Ϊfile.datʱ����ʵ�ʴ򿪵��ļ���Ϊlibfile.dat.so��������Androidϵͳ�£�
	*  �����߿��԰��մ˹��򽫱�����Դ������, ����libsĿ¼�´����apk���ڰ�װ����Щ��Դ�ļ�
	*  �ͻ������/data/data/����/libĿ¼�¡������ֱ�ӽ�dataPath������ָΪ��Ŀ¼���ɡ�
	*  @n@n
	*
	*/ 

	HCI_ERR_CODE HCIAPI hci_asr_init(
		_MUST_ _IN_ const char * pszConfig
		);

	/**  
	* @brief	��ʼ�Ự
	* @param	pszConfig		�Ự���ô���ASCII�ַ�������'\0'����;
	* @param	pnSessionId		�ɹ�ʱ���ػỰID
	* @return
	* @n
	*	<table>
	*		<tr><td>@ref HCI_ERR_NONE</td><td>�����ɹ�</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_NOT_INIT</td><td>HCI ASR��δ��ʼ��</td></tr>
	*		<tr><td>@ref HCI_ERR_PARAM_INVALID</td><td>����������Ϸ�</td></tr>
	*		<tr><td>@ref HCI_ERR_CONFIG_INVALID</td><td>������Ϸ�</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_ENGINE_FAILED</td><td>��������ʶ��ʧ��</td></tr>
	*		<tr><td>@ref HCI_ERR_TOO_MANY_SESSION</td><td>������Session������������</td></tr>
	*		<tr><td>@ref HCI_ERR_CONFIG_CAPKEY_MISSING</td><td>ȱ�ٱ����capKey������</td></tr>
	*		<tr><td>@ref HCI_ERR_URL_MISSING</td><td>�Ҳ�����Ӧ����������ַ��HCI���������ַ)</td></tr>
	*		<tr><td>@ref HCI_ERR_CONFIG_CAPKEY_NOT_MATCH</td><td>CAPKEY�뵱ǰ���治ƥ��</td></tr>
	*		<tr><td>@ref HCI_ERR_CAPKEY_NOT_FOUND</td><td>û���ҵ�ָ��������</td></tr>
	*		<tr><td>@ref HCI_ERR_LOAD_FUNCTION_FROM_DLL</td><td>���ؿ⺯��ʧ��</td></tr>
	*		<tr><td>@ref HCI_ERR_CONFIG_UNSUPPORT</td><td>�����֧��,�ƶ��ݲ�֧��grammar��ʵʱʶ��</td></tr>
	*		<tr><td>@ref HCI_ERR_CONFIG_DATAPATH_MISSING</td><td>ȱ�ٱ����dataPath������</td></tr>
	*	</table>
	*
	* @par ���ô����壺
	* ���ô�����"�ֶ�=ֵ"����ʽ������һ���ַ���������ֶ�֮����','�������ֶ������ִ�Сд��
	* ֧����+��˫·ʶ���ܣ����Ҫʹ����+��˫·ʶ����Ҫ���ض��������ú��ƶ��������ã�����֮��ʹ��'#'�ָ
	* ���磺capkey=asr.cloud.freetalk,realtime=yes\#capkey=asr.local.grammar,realtime=yes,grammartype=id,grammarid=2
	* @n@n
	* �����Ự������ã�
	*	<table>
	*		<tr>
	*			<td><b>�ֶ�</b></td>
	*			<td><b>ȡֵ��ʾ��</b></td>
	*			<td><b>ȱʡֵ</b></td>
	*			<td><b>����</b></td>
	*			<td><b>��ϸ˵��</b></td>
	*		</tr>
	*		<tr>
	*			<td>capKey</td>
	*			<td>�ַ������ο� @ref hci_asr_page </td>
	*			<td>��</td>
	*			<td>����ʶ������key</td>
	*			<td>������ÿ��sessionֻ�ܶ���һ�����������ҹ����в��ܸı䡣</td>
	*		</tr>
    *		<tr>
    *			<td>resPrefix</td>
    *			<td>�ַ������磺temp_</td>
    *			<td>��</td>
    *			<td>��Դ����ǰ׺</td>
    *			<td>���漰��������������£�����ɺ��ԡ������ͬ�Ự��Ҫʹ��ͬһ·������Դʱ����ʹ�ø��ֶζ�ͳһ·���µ���Դ��������</td>
    *		</tr>
	*		<tr>
	*			<td>intention</td>
	*			<td>�ַ������磺poi</td>
	*			<td>��</td>
	*			<td>ʶ����ͼ</td>
	*			<td>����dialogʶ������Ч��ʹ���ƶ�����ʱ���<br/>
	*				�ɴ����������Էֺŷָ���<br/>
	*				���磺intention=weather;call��  ��Ӧ�������Դ�ļ���weather_xxxx��call_xxxx<br/>
	*				��Ч���������@ref nlu_intention</td>
	*		</tr>
	*		<tr>
	*			<td>realtime</td>
	*			<td>�ַ�������Чֵ{no, yes,rt}</td>
	*			<td>no</td>
	*			<td>ʶ��ģʽ</td>
	*			<td>no,��ʵʱʶ��ģʽ<br/>
	*				yes,ʵʱʶ��ģʽ,�ƶ��﷨ʶ��֧�֣�����������֧��<br/>
	*				rt,ʵʱ����ʶ����,���ƶ�����˵asr.cloud.freetalk֧�֣�������������֧�֣�ʵʱ��������ļ���ʽ�μ� @ref hci_asr_realtime_rt "ʵʱ����"</td>
	*		</tr>
	*		<tr>
	*			<td>maxSeconds</td>
	*			<td>����������Χ[1,60]</td>
	*			<td>30</td>
	*			<td>���ʱ����������������, ����Ϊ��λ</td>
	*			<td>�����������������˳���<br/>
	*				��ʵʱʶ�𷵻�(@ref HCI_ERR_DATA_SIZE_TOO_LARGE)<br/>
	*				ʵʱʶ�𷵻�(@ref HCI_ERR_ASR_REALTIME_END)<br/>
	*				�˵�����Ϊ����maxsecondsΪ��������<br/> 
	*               ���⣺�ƶ˷�ʵʱʶ����������ԭ����ʱ�޶����ܳ���256k��</td>			
	*		</tr>
	*		<tr>
	*			<td>dialogMode</td>
	*			<td>�ַ�������Чֵ{freetalk,grammar}</td>
	*			<td>freetalk</td>
	*			<td>����dialog������ʶ��ģʽ</td>
	*			<td>������dialog��Ч���ж�����ʶ��Ĳ���ʹ�õ��Ǳ����﷨ʶ�𣬻��Ǳ�������˵ʶ��</td>			
	*		</tr>
	*		<tr>
	*			<td>netTimeout</td>
	*			<td>���糬ʱʱ�䣬��Χ[1,30]</td>
	*			<td>8</td>
	*			<td>��������ʱʱ�䣬����Ϊ��λ</td>
	*			<td>�������ӻ��������󳬹��趨ֵʱ��������ʧ�ܡ�</td>			
	*		</tr>
	*	</table>
	* @n@n
	* ���⣬���ﻹ���Դ���ʶ����������ΪĬ��������μ� hci_asr_recog() ��
	*/ 

	HCI_ERR_CODE HCIAPI hci_asr_session_start(
		_MUST_ _IN_ const char * pszConfig,
		_MUST_ _OUT_ int * pnSessionId
		);

	/**  
	* @brief	�����﷨ʶ������﷨
	* @param	pszConfig			ʶ��������ô���ASCII�ַ�������ΪNULL����'\0'����
	* @param	pszGrammarData		ʶ���﷨���ݻ��ļ��������ݱ����ԡ�\0����β, ������﷨���ݣ���������ʽΪUTF-8, ������ΪNULL
	* @param	pnGrammarId			���ص��﷨��ʶ���������� hci_asr_recog() �� hci_asr_unload_grammar() ����
	* @return
	* @n
	*	<table>
	*		<tr><td>@ref HCI_ERR_NONE</td><td>�����ɹ�</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_NOT_INIT</td><td>HCI ASR��δ��ʼ��</td></tr>
	*		<tr><td>@ref HCI_ERR_PARAM_INVALID</td><td>����������Ϸ�</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_LOAD_GRAMMAR_FAILED</td><td>�����﷨�ļ�ʧ��</td></tr>
	*		<tr><td>@ref HCI_ERR_CONFIG_INVALID</td><td>������Ϸ�</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_OPEN_GRAMMAR_FILE</td><td>��ȡ�﷨�ļ�ʧ��</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_GRAMMAR_OVERLOAD</td><td>�����﷨�����Ѵ�����ֵ256</td></tr>
	*	</table>
	*
	* @par ���ô����壺
	* ���ô�����"�ֶ�=ֵ"����ʽ������һ���ַ���������ֶ�֮����','�������ֶ������ִ�Сд��
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
    *			<td>capKey</td>
    *			<td>�ַ������ο� @ref hci_asr_page </td>
    *			<td>��</td>
    *			<td>����ʶ������key</td>
    *			<td>�����������﷨�����뿪���Ự������һ�²��Ҳ��ܸı䡣</td>
    *		</tr>
    *		<tr>
    *			<td>resPrefix</td>
    *			<td>�ַ������磺temp_</td>
    *			<td>��</td>
    *			<td>��Դ����ǰ׺</td>
    *			<td>���漰��������������£�����ɺ��ԡ������ͬ�Ự��Ҫʹ��ͬһ·������Դʱ����ʹ�ø��ֶζ�ͳһ·���µ���Դ��������</td>
    *		</tr>
	*		<tr>
	*			<td>grammarType</td>
	*			<td>�ַ�������Чֵ{wordlist, jsgf, wfst}</td>
	*			<td>jsgf</td>
	*			<td>ʹ�õ��﷨����</td>
	*			<td>ָ����������﷨��<br/>
	*				- wordlist: �� grammarData ���ݵ�����'\\r\\n'�����Ĵʱ�
	*				- jsgf: �� grammarData ���ݵ����﷨�ļ�����ʶ���﷨�ļ���ʽ��μ�����ͨ����iSpeakGrammar�﷨����˵��.pdf��
	*				- wfst: ͨ��hci_asr_save_compiled_grammar�ӿڱ�����﷨�ļ���ֻ֧�ִ��ļ�����
	*			</td>
	*		</tr>
	*		<tr>
	*			<td>isFile</td>
	*			<td>�ַ�������Чֵ{yes, no}</td>
	*			<td>no</td>
	*			<td>����������ļ��������ڴ�����</td>
	*			<td>yes��ʾpszGrammarData��ʾ����һ���﷨�ļ�����no��ʾpszGrammarData��ʾ��ֱ�����ڴ��е��﷨����</td>
	*		</tr>
	*	</table>
	*
	* @note
	* �����ñ��������ر����﷨�ļ����õ��﷨ID����ʹ�� hci_asr_recog() ����ʶ��
	* ʶ��ʱ�� hci_asr_recog() �н����õ�grammarType��Ϊid, grammarId��Ϊ�������õ����﷨ID�����ɽ���ʶ��
	* @n@n
	* ������ر����﷨��session�޹أ����session���Թ���������ص��﷨
	* 
	*/ 
	HCI_ERR_CODE HCIAPI hci_asr_load_grammar(	
		_OPT_ _IN_ const char * pszConfig,   
		_MUST_ _IN_ const char * pszGrammarData,
		_MUST_ _OUT_ unsigned int * pnGrammarId
		);

	/**  
	* @brief	�����﷨ʶ��ж���﷨
	* @param	nGrammarId			��Ҫж�ص��﷨ID���� hci_asr_load_grammar() ���
	* @return
	* @n
	*	<table>
	*		<tr><td>@ref HCI_ERR_NONE</td><td>�����ɹ�</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_NOT_INIT</td><td>HCI ASR��δ��ʼ��</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_GRAMMAR_ID_INVALID</td><td>�﷨ID��Ч</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_GRAMMAR_USING</td><td>���﷨����ʹ����</td></tr>
	*		<tr><td>@ref HCI_ERR_UNSUPPORT</td><td>ֻ�����ƶ˰汾ʱ��֧��</td></tr>
	*	</table>
	*/
	HCI_ERR_CODE HCIAPI hci_asr_unload_grammar(	
		_MUST_ _IN_ unsigned int nGrammarId
		);

	/**  
	* @brief	����������﷨�ļ���WFST��ʽ���������ļ�����ͨ��WFST���͵ķ�ʽ���룬���ڴ���﷨�ļ����Դ������﷨�����ٶȡ�
	* @param	nGrammarId			��Ҫ������﷨ID���� hci_asr_load_grammar() ���
	* @param	pcsFileName			Ҫ������﷨�ļ����ƣ�����ΪNULL��մ�
	* @return	
	* @n
	*	<table>
	*		<tr><td>@ref HCI_ERR_NONE</td><td>�����ɹ�</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_NOT_INIT</td><td>HCI ASR��δ��ʼ��</td></tr>
	*		<tr><td>@ref HCI_ERR_PARAM_INVALID</td><td>����������Ϸ�</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_GRAMMAR_ID_INVALID</td><td>�﷨ID��Ч</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_OPEN_GRAMMAR_FILE</td><td>д���﷨�ļ�ʧ��</td></tr>
	*	</table>
	*/ 
	HCI_ERR_CODE hci_asr_save_compiled_grammar(
		_MUST_ _IN_ unsigned int nGrammarId,
		_MUST_ _IN_ const char *pcsFileName
		);

	/**  
	* @brief	����ʶ��
	* @param	nSessionId			�ỰID
	* @param	pvVoiceData			Ҫʶ�����Ƶ���ݣ���ʵʱʶ���д�����Ƶʱ����Ӧ����10s��������ʹ��ʵʱʶ��
	* @param	uiVoiceDataLen		Ҫʶ�����Ƶ���ݳ��ȣ����ֽ�Ϊ��λ
	* @param	pszConfig			ʶ��������ô���ASCII�ַ�������ΪNULL����'\0'����
	* @param	pszGrammarData		ʶ���﷨���ݣ������ݱ����ԡ�\0����β,�ұ����ʽΪUTF-8,����ΪNULL
	* @param	psAsrRecogResult	ʶ������ʹ����Ϻ���ʹ�� hci_asr_free_recog_result() �����ͷ�
	* @return
	* @n
	*	<table>
	*		<tr><td>@ref HCI_ERR_NONE</td><td>�����ɹ�</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_NOT_INIT</td><td>HCI ASR��δ��ʼ��</td></tr>
	*		<tr><td>@ref HCI_ERR_PARAM_INVALID</td><td>����������Ϸ�</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_GRAMMAR_DATA_TOO_LARGE</td><td>�﷨���ݹ���(0,64K]</td></tr>
	*		<tr><td>@ref HCI_ERR_DATA_SIZE_TOO_LARGE</td><td>Encodeת�������Ƶ���ݺ��﷨�����ܴ�С��������(0,320K]����Ƶ���ݲ��˹�����������Ƶʱ����60������</td></tr>
	*		<tr><td>@ref HCI_ERR_SESSION_INVALID</td><td>�����Session�Ƿ�</td></tr>
	*		<tr><td>@ref HCI_ERR_URL_MISSING</td><td>�Ҳ�����Ӧ����������ַ��HCI���������ַ)</td></tr>
	*		<tr><td>@ref HCI_ERR_CAPKEY_NOT_FOUND</td><td>û���ҵ�ָ��������</td></tr>
	*		<tr><td>@ref HCI_ERR_SERVICE_CONNECT_FAILED</td><td>���ӷ�����ʧ�ܣ�����������Ӧ</td></tr>
	*		<tr><td>@ref HCI_ERR_SERVICE_TIMEOUT</td><td>���������ʳ�ʱ</td></tr>
	*		<tr><td>@ref HCI_ERR_SERVICE_DATA_INVALID</td><td>���������ص����ݸ�ʽ����ȷ</td></tr>
	*		<tr><td>@ref HCI_ERR_SERVICE_RESPONSE_FAILED</td><td>����������ʶ��ʧ��</td></tr>
	*		<tr><td>@ref HCI_ERR_CONFIG_INVALID</td><td>���ò�����Ч</td></tr>
	*		<tr><td>@ref HCI_ERR_CONFIG_UNSUPPORT</td><td>�����֧��</td></tr>
	*		<tr><td>@ref HCI_ERR_LOAD_CODEC_DLL</td><td>������Ƶ������ʧ��</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_OPEN_GRAMMAR_FILE</td><td>��ȡ�﷨�ļ�ʧ��</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_GRAMMAR_ID_INVALID</td><td>�﷨ID��Ч</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_LOAD_GRAMMAR_FAILED</td><td>�����﷨�ļ�ʧ��</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_ENGINE_FAILED</td><td>��������ʶ��ʧ��</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_REALTIME_WAITING</td><td>ʵʱʶ��ʱδ��⵽��Ƶĩ�ˣ������ȴ�����</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_REALTIME_END</td><td>ʵʱʶ��ʱ��⵽��Ƶĩ��</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_REALTIME_NO_VOICE_INPUT</td><td>ʵʱʶ��ʱδ�������</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_VOICE_DATA_TOO_LARGE</td><td>��ƵƬ��̫����Ӧ��(0,32K)</td></tr>
	*	</table>
	*
	* @par ���ô����壺
	* ���ô�����"�ֶ�=ֵ"����ʽ������һ���ַ���������ֶ�֮����','�������ֶ������ִ�Сд��
	* @n@n
    * ��Ƶ�������
	*	<table>
	*		<tr>
	*			<td><b>�ֶ�</b></td>
	*			<td><b>ȡֵ��ʾ��</b></td>
	*			<td><b>ȱʡֵ</b></td>
	*			<td><b>����</b></td>
	*			<td><b>��ϸ˵��</b></td>
	*		</tr>
	*		<tr>
	*			<td>audioFormat</td>
	*			<td>�ַ�������Чֵ{pcm8k16bit, ulaw8k8bit, alaw8k8bit, pcm16k16bit , ulaw16k8bit, alaw16k8bit}</td>
	*			<td>pcm16k16bit</td>
	*			<td>������������ݸ�ʽ</td>
	*			<td>����ʶ��֧�֣�pcm16k16bit<br/>
	*				�ƶ�grammar�﷨ʶ��֧�֣�pcm16k16bit��ulaw16k8bit��alaw16k8bit<br/> 
	*				�ƶ�freetalk��dialog֧�֣�pcm16k16bit��pcm8k16bit��ulaw16k8bit��ulaw8k8bit��alaw16k8bit��alaw8k8bit<br/>
	*				ע�⣬ʵʱʶ��Ķ˵����ݲ�֧�֣�alaw, ulaw<br/></td>
	*		</tr>
    *		<tr>
    *			<td>encode</td>
    *			<td>�ַ�������Чֵ{none, ulaw, alaw, speex,opus}</td>
    *			<td>none</td>
    *			<td>ʹ�õı����ʽ</td>
    *			<td>ֻ���ƶ�ʶ����Ч���Դ�����������ݽ��б��봫�䡣<br/>
    *               ����audioFormat��encode��ʹ�÷����μ�@ref codec_for_format </td>
    *		</tr> 
    *		<tr>
    *			<td>encLevel</td>
    *			<td>��������Χ[0,10]</td>
    *			<td>7</td>
    *			<td>ѹ���ȼ�</td>
    *			<td>ֻ���ƶ�ʶ����Ч��</td>
    *		</tr>
    *	</table>
    * @n@n
    * �﷨ʶ������
    *	<table>
    *		<tr>
    *			<td><b>�ֶ�</b></td>
    *			<td><b>ȡֵ��ʾ��</b></td>
    *			<td><b>ȱʡֵ</b></td>
    *			<td><b>����</b></td>
    *			<td><b>��ϸ˵��</b></td>
    *		</tr>
	*		<tr>
	*			<td>grammarType</td>
	*			<td>�ַ�������Чֵ{id, wordlist, jsgf, wfst}</td>
	*			<td>id</td>
	*			<td>ʹ�õ��﷨����</td>
	*			<td>ֻ��grammarʶ����Ч��ָ���﷨��ʽ��<br/>
	*				- id: ʹ���ƶ�Ԥ�õĻ��߱��ؼ��ص��﷨��Ž���ʶ��
	*				- wordlist: �� grammarData ���ݵ�����'\\r\\n'�����Ĵʱ����ʶ��
	*				- jsgf: �� grammarData ���ݵ����﷨�ļ�����ʶ���﷨�ļ���ʽ��μ�����ͨ����iSpeakGrammar�﷨����˵��.pdf��
	*				- wfst: ͨ��hci_asr_save_compiled_grammar�ӿڱ�����﷨�ļ���ֻ֧���ļ�����
	*			</td>
	*		</tr>
	*		<tr>
	*			<td>grammarId</td>
	*			<td>���������磺1</td>
	*			<td>��</td>
	*			<td>ʶ��ʹ�õ��﷨���</td>
	*			<td>ֻ��grammarʶ������grammarType����Ϊidʱ��Ч��<br/>
	*				- ��id�������ƶ�Ԥ�õ�ĳ���﷨Id��
	*				- Ҳ���������� hci_asr_load_grammar() ������õı��ص��﷨Id���������﷨Id�ֱ�����
	*				���ػ����ƶ˵��﷨ʶ������������֮��û�й�ϵ������ƽ̨ASR��ʶ��������Ԥ���﷨��������õ��﷨��ſ����ڿ���ʱ��ѯ��ͨ������</td>
	*		</tr>
	*		<tr>
	*			<td>isFile</td>
	*			<td>�ַ�������Чֵ{yes, no}</td>
	*			<td>no</td>
	*			<td>����������ļ��������ڴ�����</td>
	*			<td>ֻ�Ա���grammarʶ����Ч��<br/>
	*			    yes��ʾ����һ���﷨�ļ���<br/>
	*				no��ʾ��ֱ�����ڴ��е��﷨����
	*			</td>
	*		</tr>
	*	</table>
    * @n@n
    * �˵�������
    *	<table>
    *		<tr>
    *			<td><b>�ֶ�</b></td>
    *			<td><b>ȡֵ��ʾ��</b></td>
    *			<td><b>ȱʡֵ</b></td>
    *			<td><b>����</b></td>
    *			<td><b>��ϸ˵��</b></td>
    *		</tr>
    *		<tr>
    *			<td>vadSwitch</td>
    *			<td>�ַ�������Чֵ{yes��no}</td>
    *			<td>yes</td>
    *			<td>ʶ��ǰ���ж˵���</td>
    *			<td>���Ϊyes�����˵��⣬���no��رն˵���</td>
    *		</tr> 
	*		<tr>
	*			<td>vadHead</td>
	*			<td>����������Χ[0,30000]</td>
	*			<td>10000</td>
	*			<td>��ʼ�ľ�����������</td>
	*			<td>��������ʼ�ľ�����������ָ���ĺ�����ʱ����Ϊû�м�⵽����<br/>
	*				�����ֵΪ0�����ʾ�����������</td>
	*		</tr> 
	*		<tr>
	*			<td>vadTail</td>
	*			<td>����������Χ[0,30000]</td>
	*			<td>500</td>
	*			<td>�˵����ĩβ������</td>
	*			<td>��⵽�����������ݳ��־������Ҿ���ʱ�䳬������ָ���ĺ�����ʱ����Ϊ��������<br/>
	*				�����ֵΪ0�����ʾ������ĩ�˼��</td>
	*		</tr>
	*		<tr>
	*			<td>vadSeg</td>
	*			<td>����������Χ[0,30000]</td>
	*			<td>500</td>
	*			<td>�˵���ķֶμ��������</td>
	*			<td>Ŀǰֻ���ƶ�����˵ʵʱ����ģʽ��Ч��<br/>
	*				��⵽�����������ݳ��־������Ҿ���ʱ�䳬������ָ���ĺ�����ʱ����Ϊ��Ƶ���зֶ�<br/>
	*				�����ֵΪ0�����ô��ڵ���vadtail�����ʾ�����зֶ�</td>
	*		</tr>
    *		<tr>
    *			<td>vadThreshold</td>
    *			<td>����������Χ[0,100]</td>
    *			<td>10</td>
    *			<td>�˵�������������</td>
    *			<td>�˵������������ã���ֵԽСԽ����</td>
    *		</tr>
    *	</table>
	*/
#ifndef PRIVATE_CLOUD__
	/**
    * @n@n
    * ʶ�����������
    *	<table>
    *		<tr>
    *			<td><b>�ֶ�</b></td>
    *			<td><b>ȡֵ��ʾ��</b></td>
    *			<td><b>ȱʡֵ</b></td>
    *			<td><b>����</b></td>
    *			<td><b>��ϸ˵��</b></td>
    *		</tr>
    *		<tr>
    *			<td>domain</td>
    *			<td>�ַ������磺common, music, poi</td>
    *			<td>��</td>
    *			<td>ʶ��ʹ�õ�����</td>
    *			<td>ʹ��ָ�������ģ�ͽ���ʶ��<br/>
    *				- �����ѡ��������õ���������ڿ���ʱ��ѯ��ͨ������<br/>
	*				�ر�ģ�ʹ�ù���������asr.cloud.freetalk.music��asr.cloud.freetalk.poiʱ<br/>
	*				��Ҫ�����Ӧ��domain����"music" �� "poi"</td>
    *		</tr>
    *		<tr>
    *			<td>addPunc</td>
    *			<td>�ַ�������Чֵ{yes, no}</td>
    *			<td>no</td>
    *			<td>�Ƿ���ӱ��</td>
    *			<td>yes��ʾʶ���������ӱ�㣬no��ʾ����ӱ��</td>
    *		</tr>
    *		<tr>
    *			<td>candNum</td>
    *			<td>����������Χ[1,10]</td>
    *			<td>10</td>
    *			<td>ʶ���ѡ�������</td>
    *			<td></td>
    *		</tr>
	*		<tr>
	*			<td>needContent</td>
	*			<td>�ַ�������Чֵ{yes, no}</td>
	*			<td>yes</td>
	*			<td>�Ƿ���Ҫ��ͼʶ������</td>
	*			<td>����dialogʶ������Ч��<br/>
	*				ָ���Ƿ���Ҫ��ͼʶ�����ݣ�����ͼʶ����ȡ��Ӧ�����ݻ�𰸡�</td>
	*		</tr>
	*		<tr>
	*			<td>context</td>
	*			<td>�ַ�������Ч��{yes,no}</td>
	*			<td>yes</td>
	*			<td>�Ƿ��������Ĺ���</td>
	*			<td>����dialogʶ������Ч��<br/>
	*				yes ʹ�������ģ�ʹNLU�ܼ�ס�Ի����龰<br/>
	*				no ��ʹ�������ġ�
	*		</tr>
	*	</table>
	* @n@n
	* ����û�ж�����������ʹ�� session_start ʱ�Ķ��塣��� session_start ʱҲû�ж��壬��ʹ��ȱʡֵ
	* ���⣬���ﻹ���Դ���˵��������� ��Ϊʵʱʶ��ʱ���ж˵�������á�
	*/ 
#else
	/**
    * @n@n
    * ʶ�����������
    *	<table>
    *		<tr>
    *			<td><b>�ֶ�</b></td>
    *			<td><b>ȡֵ��ʾ��</b></td>
    *			<td><b>ȱʡֵ</b></td>
    *			<td><b>����</b></td>
    *			<td><b>��ϸ˵��</b></td>
    *		</tr>
    *		<tr>
    *			<td>addPunc</td>
    *			<td>�ַ�������Чֵ{yes, no}</td>
    *			<td>no</td>
    *			<td>�Ƿ���ӱ��</td>
    *			<td>yes��ʾʶ���������ӱ�㣬no��ʾ����ӱ��</td>
    *		</tr>
	*		<tr>
	*			<td>candNum</td>
	*			<td>����������Χ[1,10]</td>
	*			<td>10</td>
	*			<td>ʶ���ѡ�������</td>
	*			<td></td>
	*		</tr>
	*		<tr>
	*			<td>intention</td>
	*			<td>�ַ������磺poi</td>
	*			<td>��</td>
	*			<td>ʶ����ͼ</td>
	*			<td>�����ƶ�dialogʶ������Ч��<br/>
	*				- �ƶ�ʶ����Զ���ʶ����������ͼ������������json��ʽ����ͼ���������ȡֵ������Ϣ����ѯ��ͨ������˾</td>
	*		</tr>
	*		<tr>
	*			<td>needContent</td>
	*			<td>�ַ�������Чֵ{yes, no}</td>
	*			<td>yes</td>
	*			<td>�Ƿ���Ҫ��ͼʶ������</td>
	*			<td>�����ƶ�dialogʶ������Ч��<br/>
	*				ָ���Ƿ���Ҫ��ͼʶ�����ݣ�����ͼʶ����ȡ��Ӧ�����ݻ�𰸡�</td>
	*		</tr>
	*		<tr>
	*			<td>property</td>
	*			<td>�ַ��������磺chinese_8k_common</td>
	*			<td>��</td>
	*			<td>ȡֵ������lang��modeltype������domain��ϡ� </td>
	*			<td>��ѡ����Чȡֵ��Χ���£�<br/>
	*				lang: chinese��english <br/>
	*				modeltype: 8k(8kģ��), 16k(16kģ��) <br/>
	*				domain: common(ͨ������), poi(��������), music(��������), telecom(��������) <br/>
	*		</td>
	*		</tr>
	*	</table>
	* @n@n
	* ����û�ж�����������ʹ�� session_start ʱ�Ķ��塣��� session_start ʱҲû�ж��壬��ʹ��ȱʡֵ
	* ���⣬���ﻹ���Դ���˵��������� ��Ϊʵʱʶ��ʱ���ж˵�������á�
	*/
#endif
	HCI_ERR_CODE HCIAPI hci_asr_recog(	
		_MUST_ _IN_ int nSessionId,
		_MUST_ _IN_ void * pvVoiceData,
		_MUST_ _IN_ unsigned int uiVoiceDataLen,
		_OPT_ _IN_ const char * pszConfig,   
		_OPT_ _IN_ const char * pszGrammarData,
		_MUST_ _OUT_ ASR_RECOG_RESULT * psAsrRecogResult
		);

	/**  
	* @brief	�ͷ�����ʶ�����ڴ�
	* @param	psAsrRecogResult	��Ҫ�ͷŵ�ʶ�����ڴ�
	* @return	
	* @n
	*	<table>
	*		<tr><td>@ref HCI_ERR_NONE</td><td>�����ɹ�</td></tr>
	*		<tr><td>@ref HCI_ERR_PARAM_INVALID</td><td>����������Ϸ�</td></tr>
	*	</table>
	*/ 
	HCI_ERR_CODE HCIAPI hci_asr_free_recog_result(	
		_MUST_ _IN_ ASR_RECOG_RESULT * psAsrRecogResult
		);

	/**  
	* @brief	�ύȷ�Ͻ������
	* @param	nSessionId			�ỰID
	* @param	pAsrConfirmItem		Ҫ�ύ��ȷ�Ͻ��
	* @return
	* @n
	*	<table>
	*		<tr><td>@ref HCI_ERR_NONE</td><td>�����ɹ�</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_NOT_INIT</td><td>HCI ASR��δ��ʼ��</td></tr>
	*		<tr><td>@ref HCI_ERR_PARAM_INVALID</td><td>����������Ϸ�</td></tr>
	*		<tr><td>@ref HCI_ERR_SESSION_INVALID</td><td>�����Session�Ƿ�</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_CONFIRM_NO_TASK</td><td>û�п������ύ������������δʶ�𣬾͵��ñ�����</td></tr>
	*		<tr><td>@ref HCI_ERR_DATA_SIZE_TOO_LARGE</td><td>ȷ�����ݳ�����Χ(0,2K]</td></tr>
	*		<tr><td>@ref HCI_ERR_SERVICE_CONNECT_FAILED</td><td>���ӷ�����ʧ�ܣ�����������Ӧ</td></tr>
	*		<tr><td>@ref HCI_ERR_SERVICE_TIMEOUT</td><td>���������ʳ�ʱ</td></tr>
	*		<tr><td>@ref HCI_ERR_SERVICE_DATA_INVALID</td><td>���������ص����ݸ�ʽ����ȷ</td></tr>
	*		<tr><td>@ref HCI_ERR_SERVICE_RESPONSE_FAILED</td><td>����������ʶ��ʧ��</td></tr>
	*		<tr><td>@ref HCI_ERR_UNSUPPORT</td><td>����ʶ��֧��ȷ�Ͻ��</td></tr>
	*	</table>
	*/ 
	HCI_ERR_CODE HCIAPI hci_asr_confirm(	
		_MUST_ _IN_ int nSessionId,
		_MUST_ _IN_ ASR_CONFIRM_ITEM * pAsrConfirmItem
		);

	/**  
	* @brief	�����Ự
	* @param	nSessionId				�ỰID
	* @return	������
	* @return
	* @n
	*	<table>
	*		<tr><td>@ref HCI_ERR_NONE</td><td>�����ɹ�</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_NOT_INIT</td><td>HCI ASR��δ��ʼ��</td></tr>
	*		<tr><td>@ref HCI_ERR_SESSION_INVALID</td><td>�����Session�Ƿ�</td></tr>
	*	</table>
	*/
	HCI_ERR_CODE HCIAPI hci_asr_session_stop(
		_MUST_ _IN_ int nSessionId
		);

	/**  
	* @brief	����ASR���� ����ʼ��
	* @return	������
	* @return
	* @n
	*	<table>
	*		<tr><td>@ref HCI_ERR_NONE</td><td>�����ɹ�</td></tr>
	*		<tr><td>@ref HCI_ERR_ASR_NOT_INIT</td><td>HCI ASR��δ��ʼ��</td></tr>
	*		<tr><td>@ref HCI_ERR_ACTIVE_SESSION_EXIST</td><td>����δstop��Sesssion���޷�����</td></tr>
	*	</table>
	*/ 
	HCI_ERR_CODE HCIAPI hci_asr_release();

    /* @} */
    //////////////////////////////////////////////////////////////////////////
	/* @} */
	//////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
};
#endif


#endif
