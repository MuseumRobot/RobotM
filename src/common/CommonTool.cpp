#include "CommonTool.h"
#include <stdio.h>
#ifdef _WIN32
#include <windows.h>
#endif

#ifdef __LINUX__
#include <stdlib.h> 
#endif

namespace HciExampleComon{

    void SetSpecialConsoleTextAttribute()
    {
#ifndef _WIN32_WCE
#ifdef _WIN32
        SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY );
#else
        printf( "\033[1;32;" );
#endif
#endif
    }

    void SetOriginalConsoleTextAttribute()
    {
#ifndef _WIN32_WCE
#ifdef _WIN32
        SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
#else
        printf( "\033[0;39m" );
#endif
#endif
    }

    void PrintUtf8String(char *pUTF8Str)
    {
#ifdef _WIN32
        unsigned char* pszGBK;
        UTF8ToGBK( (unsigned char*)pUTF8Str, (unsigned char**)&pszGBK);
        printf( "%s", pszGBK );
        FreeConvertResult( pszGBK );
#else
        printf( "%s", pUTF8Str );
#endif
    }

#ifdef _WIN32
int WTC(unsigned char * pUTF8Str,unsigned char * pGBKStr,int nGBKStrLen);
int CTW(unsigned char * lpGBKStr,unsigned char * lpUTF8Str,int nUTF8StrLen);
#endif

int UTF8ToGBK(unsigned char * pUTF8Str,unsigned char ** pGBKStr)
{
#ifdef _WIN32
    int nRetLen = 0;
    nRetLen = WTC(pUTF8Str,NULL,0);
    (* pGBKStr) = (unsigned char *)malloc((nRetLen + 1)*sizeof(char));
    if((* pGBKStr) == NULL)
        return 0;
    nRetLen = WTC(pUTF8Str,(* pGBKStr),nRetLen);
    return nRetLen;
#else
    //TODO
#endif
}

int GBKToUTF8(unsigned char * pGBKStr,unsigned char ** pUTF8Str)
{
#ifdef _WIN32
    int nRetLen = 0;
    nRetLen = CTW(pGBKStr,NULL,0);
    (* pUTF8Str) = (unsigned char *)malloc((nRetLen + 1)*sizeof(char));
    if((* pUTF8Str) == NULL)
        return 0;
    nRetLen = CTW(pGBKStr,(* pUTF8Str),nRetLen);
    return nRetLen;
#else
    //TODO
#endif
}


void FreeConvertResult(unsigned char * pConvertResult)
{
    if (pConvertResult)
    {
        free(pConvertResult);
        pConvertResult = NULL;
    }
    
}

#ifdef _WIN32

int WTC(unsigned char * pUTF8Str,unsigned char * pGBKStr,int nGBKStrLen)
{
    wchar_t * lpUnicodeStr = NULL;
    int nRetLen = 0;

    if(!pUTF8Str)  //���GBK�ַ���ΪNULL������˳�
        return 0;

    nRetLen = MultiByteToWideChar(CP_UTF8,0,(char *)pUTF8Str,-1,NULL,0);  //��ȡת����Unicode���������Ҫ���ַ��ռ䳤��
    //lpUnicodeStr = new WCHAR[nRetLen + 1];  //ΪUnicode�ַ����ռ�
    lpUnicodeStr = (WCHAR *)malloc((nRetLen + 1) * sizeof(WCHAR));  //ΪUnicode�ַ����ռ�
    if(lpUnicodeStr == NULL)
        return 0;
    memset(lpUnicodeStr,0,(nRetLen + 1) * sizeof(WCHAR));
    nRetLen = MultiByteToWideChar(CP_UTF8,0,(char *)pUTF8Str,-1,lpUnicodeStr,nRetLen);  //ת����Unicode����
    if(!nRetLen)  //ת��ʧ��������˳�
        return 0;

    nRetLen = WideCharToMultiByte(CP_ACP,0,lpUnicodeStr,-1,NULL,0,NULL,NULL);  //��ȡת����UTF8���������Ҫ���ַ��ռ䳤��

    if(!pGBKStr)  //���������Ϊ���򷵻�ת������Ҫ�Ŀռ��С
    {
        if(lpUnicodeStr)       
            //delete []lpUnicodeStr;
            free(lpUnicodeStr);
        return nRetLen;
    }

    if(nGBKStrLen < nRetLen)  //���������������Ȳ������˳�
    {
        if(lpUnicodeStr)
            //delete []lpUnicodeStr;
            free(lpUnicodeStr);
        return 0;
    }

    nRetLen = WideCharToMultiByte(CP_ACP,0,lpUnicodeStr,-1,(char *)pGBKStr,nGBKStrLen,NULL,NULL);  //ת����UTF8����

    if(lpUnicodeStr)
        //delete []lpUnicodeStr;
        free(lpUnicodeStr);

    return nRetLen;
}

int CTW(unsigned char * lpGBKStr,unsigned char * lpUTF8Str,int nUTF8StrLen)
{
    wchar_t * lpUnicodeStr = NULL;
    int nRetLen = 0;

    if(!lpGBKStr)  //���GBK�ַ���ΪNULL������˳�
        return 0;

    nRetLen = MultiByteToWideChar(CP_ACP,0,(char *)lpGBKStr,-1,NULL,0);  //��ȡת����Unicode���������Ҫ���ַ��ռ䳤��
    //lpUnicodeStr = new WCHAR[nRetLen + 1];  //ΪUnicode�ַ����ռ�
    lpUnicodeStr = (WCHAR *)malloc((nRetLen + 1) * sizeof(WCHAR));  //ΪUnicode�ַ����ռ�
    if(lpUnicodeStr == NULL)
        return 0;


    nRetLen = MultiByteToWideChar(CP_ACP,0,(char *)lpGBKStr,-1,lpUnicodeStr,nRetLen);  //ת����Unicode����
    if(!nRetLen)  //ת��ʧ��������˳�
        return 0;

    nRetLen = WideCharToMultiByte(CP_UTF8,0,lpUnicodeStr,-1,NULL,0,NULL,NULL);  //��ȡת����UTF8���������Ҫ���ַ��ռ䳤��

    if(!lpUTF8Str)  //���������Ϊ���򷵻�ת������Ҫ�Ŀռ��С
    {
        if(lpUnicodeStr)       
            //delete []lpUnicodeStr;
            free(lpUnicodeStr);
        return nRetLen;
    }

    if(nUTF8StrLen < nRetLen)  //���������������Ȳ������˳�
    {
        if(lpUnicodeStr)
            //delete []lpUnicodeStr;
            free(lpUnicodeStr);
        return 0;
    }

    nRetLen = WideCharToMultiByte(CP_UTF8,0,lpUnicodeStr,-1,(char *)lpUTF8Str,nUTF8StrLen,NULL,NULL);  //ת����UTF8����

    if(lpUnicodeStr)
        //delete []lpUnicodeStr;
        free(lpUnicodeStr);
    return nRetLen;
}

#endif

}
