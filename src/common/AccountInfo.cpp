#include "AccountInfo.h"
#include <vector>
#include <fstream>
using std::ifstream;
using std::vector;

#ifdef __LINUX__
#include <string.h>
#define stricmp strcasecmp
#else
#define stricmp _stricmp
#endif


//�ַ���trim����
string trim(const string& str) {
    string t = str;
    t.erase(0, t.find_first_not_of(" \t\n\r"));
    t.erase(t.find_last_not_of(" \t\n\r") + 1);
    return t;
}  

//�ַ����ָ��
vector<string> split(string str,string pattern)
{
    string::size_type pos;
    vector<std::string> result;
    str+=pattern;//��չ�ַ����Է������
    int size=str.size();

    for(int i=0; i<size; i++)
    {
        pos=str.find(pattern,i);
        if(pos<size)
        {
            std::string s=str.substr(i,pos-i);
			s=trim(s);
            result.push_back(s);
            i=pos+pattern.size()-1;
        }
    }
    return result;
}

#define APP_KEY_STR "appKey"
#define DEVELOPER_KEY_STR "developerKey"
#define CLOUD_URL_STR "cloudUrl"
#define CAP_KEY_STR "capKey"

AccountInfo *AccountInfo::instance_ = NULL;
AccountInfo *AccountInfo::GetInstance()
{
    if (instance_ == NULL)
    {
        instance_ = new AccountInfo;
    }
    return instance_;
}

void AccountInfo::ReleaseInstance()
{
    if (instance_ == NULL)
    {
        delete instance_;
		instance_ = NULL;
    }
}

AccountInfo::AccountInfo()
{
    app_key_ = "";
    developer_key_ = "";
    cloud_url_ = "";
    cap_key_ = "";
	
	string path_prefix = "../..";
#ifdef _WIN32_WCE
	path_prefix = "/SDMMC";
#endif

	test_data_path_ = path_prefix + "/testdata";
	data_path_ = path_prefix + "/data";
	auth_path_ = path_prefix + "/bin";
	logfile_path_ = path_prefix + "/bin";
}

AccountInfo::~AccountInfo()
{

}

//�˴�ʵ��һ���򵥵������ļ���ȡ�����������߿�������ѡ���Ƿ�ʹ��
bool AccountInfo::LoadFromFile(const string &account_info_file)
{
    //���ļ�
    ifstream fin;
    fin.open(account_info_file.c_str());
    if (!fin)
    {
        printf("get account info failed \n\t may be the file %s not exist!\n",account_info_file.c_str());
        return false;
    }

    //���б���
    string line;
    while(getline(fin,line))
    {
        line = trim(line);
        //Ϊ�ջ����ַ�Ϊ"#",�����
        if (line.empty() || line[0] == '#' )
        {
            continue;
        }

        vector<string> key_value = split(line,"=");
        if (stricmp(key_value[0].c_str(),APP_KEY_STR) == 0)
        {
            app_key_ = key_value[1];
        }
        if (stricmp(key_value[0].c_str(),DEVELOPER_KEY_STR) == 0)
        {
            developer_key_ = key_value[1];
        }
        if (stricmp(key_value[0].c_str(),CLOUD_URL_STR) == 0)
        {
            cloud_url_ = key_value[1];
        }
        if (stricmp(key_value[0].c_str(),CAP_KEY_STR) == 0)
        {
            cap_key_ = key_value[1];
        }
    }

    //�ر��ļ�
    fin.close();

    //��Ч���ж�
    if (!IsValid())
    {
        printf("get account info failed \n\t \
               account info(%s,%s,%s,%s) \n\t \
               some record info may be missed,\
               please check the file %s!\n",
            APP_KEY_STR,DEVELOPER_KEY_STR,CLOUD_URL_STR,CAP_KEY_STR,account_info_file.c_str());
        return false;
    }
    return true;
}

bool AccountInfo::LoadFromCode()
{
    //�����и�ֵ���ɣ�����AccountInfo.txt�ļ�
    //��ʱ��Ҫ�ڴ˴���д��ȷ��Ӧ���˺���Ϣ
    app_key_ = "";
    developer_key_ = "";
    cloud_url_ = "";
    cap_key_ = "";

    //��Ч���ж�
    if (!IsValid())
    {
        printf("get account info failed \n\t \
               account info(%s,%s,%s,%s) \n\t \
               some record info may be missed!\n",
               APP_KEY_STR,DEVELOPER_KEY_STR,CLOUD_URL_STR,CAP_KEY_STR);
        return false;
    }
    return false;
}

bool AccountInfo::IsValid()
{
    if (app_key_.empty() || developer_key_.empty() || cloud_url_.empty()||cap_key_.empty())
    {
        return false;
    }
    return true;
}

string AccountInfo::app_key()
{
    return app_key_;
}

string AccountInfo::developer_key()
{
    return developer_key_;
}

string AccountInfo::cloud_url()
{
    return cloud_url_;
}

string AccountInfo::auth_path()
{
    return auth_path_;
}

string AccountInfo::logfile_path()
{
    return logfile_path_;
}

string AccountInfo::cap_key()
{
    return cap_key_;
}

string AccountInfo::data_path()
{
    return data_path_;
}

string AccountInfo::test_data_path()
{
    return test_data_path_;
}

