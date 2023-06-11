# make a mosaic with 19 videos
# 1 2 3 4 5
# 6 7 8 9 10
# 11 12 13 14 15
# 16 17 18 19 20


ffmpeg -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video1.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video2.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video3.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video4.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video5.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video6.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video7.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video8.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video9.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video10.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video11.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video12.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video13.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video14.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video15.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video16.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video17.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video18.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video19.mp4 \
       -i /home/as/TSlam/eval/script/test/mosaic_video/src_vid/video10.mp4 \
-filter_complex \
"[0:v]setpts=PTS-STARTPTS, scale=320x240 [a0]; \
[1:v]setpts=PTS-STARTPTS, scale=320x240 [a1]; \
[2:v]setpts=PTS-STARTPTS, scale=320x240 [a2]; \
[3:v]setpts=PTS-STARTPTS, scale=320x240 [a3]; \
[4:v]setpts=PTS-STARTPTS, scale=320x240 [a4]; \
[5:v]setpts=PTS-STARTPTS, scale=320x240 [a5]; \
[6:v]setpts=PTS-STARTPTS, scale=320x240 [a6]; \
[7:v]setpts=PTS-STARTPTS, scale=320x240 [a7]; \
[8:v]setpts=PTS-STARTPTS, scale=320x240 [a8]; \
[9:v]setpts=PTS-STARTPTS, scale=320x240 [a9]; \
[10:v]setpts=PTS-STARTPTS, scale=320x240 [a10]; \
[11:v]setpts=PTS-STARTPTS, scale=320x240 [a11]; \
[12:v]setpts=PTS-STARTPTS, scale=320x240 [a12]; \
[13:v]setpts=PTS-STARTPTS, scale=320x240 [a13]; \
[14:v]setpts=PTS-STARTPTS, scale=320x240 [a14]; \
[15:v]setpts=PTS-STARTPTS, scale=320x240 [a15]; \
[16:v]setpts=PTS-STARTPTS, scale=320x240 [a16]; \
[17:v]setpts=PTS-STARTPTS, scale=320x240 [a17]; \
[18:v]setpts=PTS-STARTPTS, scale=320x240 [a18]; \

[a0][a1][a2][a3][a4][a5][a6][a7][a8][a9][a10][a11][a12][a13][a14][a15][a16][a17][a18]xstack=inputs=19:layout=0_0|w0_0|w0+w1_0|w0+w1+w2_0|w0+w1+w2+w3_0|0_h0|w0_h0|w0+w1_h0|w0+w1+w2_h0|w0+w1+w2+w3_h0|0_h0+h1|w0_h0+h1|w0+w1_h0+h1|w0+w1+w2_h0+h1|w0+w1+w2+w3_h0+h1|0_h0+h1+h2|w0_h0+h1+h2|w0+w1_h0+h1+h2|w0+w1+w2_h0+h1+h2|w0+w1+w2+w3_h0+h1+h2[out]" \
-map "[out]" -c:v libx264 -crf 23 -preset veryfast -c:a copy -shortest /home/as/TSlam/eval/script/test/mosaic_video/mosaic.mp4
