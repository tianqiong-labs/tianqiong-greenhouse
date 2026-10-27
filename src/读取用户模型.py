"""通过 SolidWorks COM 打开用户的装配体，导出 STEP。

只读打开、静默、不改原文件。OpenDoc6 / SaveAs 的 ByRef 参数必须用 VARIANT。
"""
import os, sys, time
import pythoncom
import win32com.client as win32

SRC = r"C:\Users\12753\Desktop\智能农庄校赛\装配体.SLDASM"
OUT_DIR = r"C:\Users\12753\Documents\田穹校赛\src\out\user_model"
os.makedirs(OUT_DIR, exist_ok=True)
OUT_STEP = os.path.join(OUT_DIR, "用户装配体.step")

if not os.path.exists(SRC):
    print("SRC missing:", SRC); sys.exit(2)


def byref0():
    return win32.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)


try:
    sw = win32.gencache.EnsureDispatch("SldWorks.Application")
    print("gencache OK")
except Exception as _e:
    print("gencache 失败，用 Dispatch:", _e)
    sw = win32.Dispatch("SldWorks.Application")
sw.Visible = True

doc = None
try:
    e, w = byref0(), byref0()
    doc = sw.OpenDoc6(SRC, 2, 1, "", e, w)   # 2=装配体 1=静默
except Exception as ex:
    print("OpenDoc6 失败:", ex)
    try:
        doc = sw.OpenDoc(SRC, 2)
        print("回退 OpenDoc 成功")
    except Exception as ex2:
        print("OpenDoc 也失败:", ex2)

if not doc:
    print("无法打开文档，停止")
    sys.exit(3)

title = doc.GetTitle if isinstance(doc.GetTitle, str) else doc.GetTitle()
print("已打开:", title)
try:
    doc.ViewZoomtofit2()
except Exception:
    pass

ok = False
for attempt, args in enumerate([
    (OUT_STEP, 0, 1, None),
    (OUT_STEP, 0, 0, None),
]):
    try:
        r = doc.SaveAs3(args[0], 0, 1)
        print(f"SaveAs3#{attempt} -> {r}")
    except Exception as ex:
        print(f"SaveAs3#{attempt} 抛错:", ex)
    if os.path.exists(OUT_STEP) and os.path.getsize(OUT_STEP) > 1000:
        ok = True
        break

if ok:
    print("STEP:", OUT_STEP, os.path.getsize(OUT_STEP), "bytes")
else:
    print("STEP 未生成")
