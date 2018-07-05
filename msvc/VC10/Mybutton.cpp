#include "stdafx.h"

#include"Mybutton.h"

void CMybutton::DrawItem(LPDRAWITEMSTRUCT lpDrawItemStruct){

	CDC dc;
dc.Attach(lpDrawItemStruct->hDC);//CDC类里的ATTACH函数用于把HDC转换成CDC
UINT state=lpDrawItemStruct->itemState;
CRect ButtonRect;
GetClientRect(&ButtonRect);//调用按钮类里的GetWindowRect函数获取按钮区域大小
CDC memDC;
memDC.CreateCompatibleDC(&dc);//创建兼容DC
CBitmap bmp;
if(state&ODS_SELECTED)//如果按钮按下
{
bmp.LoadBitmap(IDB_SELECT);//装载按下图片
}
else
{
bmp.LoadBitmap(IDB_NORMAL);
}
BITMAP bmpInfo;
bmp.GetBitmap(&bmpInfo);//获取位图信息
memDC.SelectObject(&bmp);//选入图片
dc.StretchBlt(0,0,ButtonRect.right,ButtonRect.bottom,&memDC,
0,0,bmpInfo.bmWidth,bmpInfo.bmHeight,SRCCOPY);//显示图片
CString str;
GetWindowText(str);//获取按钮文本
dc.SetBkMode(TRANSPARENT);//设置文字背景透明
dc.DrawText(str,&ButtonRect,DT_CENTER|DT_VCENTER|DT_SINGLELINE);//输出按钮文本 memDC.DeleteDC();
bmp.DeleteObject();

}

