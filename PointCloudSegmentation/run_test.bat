@echo off
@REM for /D %%G in ("testing_data\bg1\*") do (
@REM     C:/Users/koala/anaconda3/envs/realsense/python.exe c:/Users/koala/OneDrive/Desktop/PointCloudGeneration/imgs2pc.py %%G
@REM )
@REM for /D %%G in ("testing_data\bg2\*") do (
@REM     C:/Users/koala/anaconda3/envs/realsense/python.exe c:/Users/koala/OneDrive/Desktop/PointCloudGeneration/imgs2pc.py %%G
@REM )

@REM for /D %%G in ("testing_data\bg1\*") do (
@REM     C:/Users/koala/anaconda3/envs/realsense/python.exe c:/Users/koala/OneDrive/Desktop/PointCloudGeneration/segment.py %%G
@REM )
@REM for /D %%G in ("testing_data\bg2\*") do (
@REM     C:/Users/koala/anaconda3/envs/realsense/python.exe c:/Users/koala/OneDrive/Desktop/PointCloudGeneration/segment.py %%G
@REM )

@REM for /D %%G in ("testing_data\bg1\*") do (
@REM     C:/Users/koala/anaconda3/envs/realsense/python.exe c:/Users/koala/OneDrive/Desktop/PointCloudGeneration/segment2.py %%G
@REM )
@REM for /D %%G in ("testing_data\bg2\*") do (
@REM     C:/Users/koala/anaconda3/envs/realsense/python.exe c:/Users/koala/OneDrive/Desktop/PointCloudGeneration/segment2.py %%G
@REM )

for /D %%G in ("output\bg1\*") do (
    C:/Users/koala/anaconda3/envs/realsense/python.exe c:/Users/koala/OneDrive/Desktop/PointCloudGeneration/calc_iou.py %%G
)
for /D %%G in ("output\bg2\*") do (
    C:/Users/koala/anaconda3/envs/realsense/python.exe c:/Users/koala/OneDrive/Desktop/PointCloudGeneration/calc_iou.py %%G
)

@REM for /D %%G in ("output\bg1\*") do (
@REM     C:/Users/koala/anaconda3/envs/realsense/python.exe c:/Users/koala/OneDrive/Desktop/PointCloudGeneration/render_pc_as_img.py %%G
@REM )
@REM for /D %%G in ("output\bg2\*") do (
@REM     C:/Users/koala/anaconda3/envs/realsense/python.exe c:/Users/koala/OneDrive/Desktop/PointCloudGeneration/render_pc_as_img.py %%G
@REM )

for /D %%G in ("output\bg1\*") do (
    C:/Users/koala/anaconda3/envs/realsense/python.exe c:/Users/koala/OneDrive/Desktop/PointCloudGeneration/calc_confusion.py %%G
)
for /D %%G in ("output\bg2\*") do (
    C:/Users/koala/anaconda3/envs/realsense/python.exe c:/Users/koala/OneDrive/Desktop/PointCloudGeneration/calc_confusion.py %%G
)

pause