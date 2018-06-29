#include "stdafx.h"

#include"Mybutton.h"

void CMybutton::DrawItem(LPDRAWITEMSTRUCT lpDrawItemStruct){

	CDC dc;
dc.Attach(lpDrawItemStruct->hDC);//CDC�����ATTACH�������ڰ�HDCת����CDC
UINT state=lpDrawItemStruct->itemState;
CRect ButtonRect;
GetClientRect(&ButtonRect);//���ð�ť�����GetWindowRect������ȡ��ť�����С
CDC memDC;
memDC.CreateCompatibleDC(&dc);//��������DC
CBitmap bmp;
if(state&ODS_SELECTED)//�����ť����
{
bmp.LoadBitmap(IDB_SELECT);//װ�ذ���ͼƬ
}
else
{
bmp.LoadBitmap(IDB_NORMAL);
}
BITMAP bmpInfo;
bmp.GetBitmap(&bmpInfo);//��ȡλͼ��Ϣ
memDC.SelectObject(&bmp);//ѡ��ͼƬ
dc.StretchBlt(0,0,ButtonRect.right,ButtonRect.bottom,&memDC,
0,0,bmpInfo.bmWidth,bmpInfo.bmHeight,SRCCOPY);//��ʾͼƬ
CString str;
GetWindowText(str);//��ȡ��ť�ı�
dc.SetBkMode(TRANSPARENT);//�������ֱ���͸��
dc.DrawText(str,&ButtonRect,DT_CENTER|DT_VCENTER|DT_SINGLELINE);//�����ť�ı� memDC.DeleteDC();
bmp.DeleteObject();

}

