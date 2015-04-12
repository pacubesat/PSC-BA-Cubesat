Using Raspberry Pi Micro-Computer with Avconv Package


.barchrc file 


bash .1 &
bash .2 &


.1 file (image capturing)

for i in {1..100}
do
avconv -f video4linux2 	-i /dev/video0	-vframe 1 $i.jpeg
sleep 0.5;
done

.2  file (video Recording)

while true;
do
avconv -f video4linux2 	-i /dev/video1	-f mp4 video.mp4
sleep 1;
done

