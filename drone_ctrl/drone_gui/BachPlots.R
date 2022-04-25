DATANY <- `flight2video`;
Time_s <- as.numeric(as.character(DATANY$V1)) / 1000;
Arm <- as.numeric(as.character(DATANY$V2));
Flight_state <- as.numeric(as.character(DATANY$V3));
Height_ref <- as.numeric(as.character(DATANY$V4));
Roll_ref <- as.numeric(as.character(DATANY$V5));
Pitch_ref <- as.numeric(as.character(DATANY$V6));
Yaw_ref <- as.numeric(as.character(DATANY$V7));
Height_mix <- as.numeric(as.character(DATANY$V8));
Roll_mix <- as.numeric(as.character(DATANY$V9)) / 100;
Pitch_mix <- as.numeric(as.character(DATANY$V10)) / 100;
Yaw_mix <- as.numeric(as.character(DATANY$V11)) / 100;
Height_pos <- as.numeric(as.character(DATANY$V12));
Roll_angle <- as.numeric(as.character(DATANY$V13));
Pitch_angle <- as.numeric(as.character(DATANY$V14));
Yaw_vel <- as.numeric(as.character(DATANY$V15)) / 10;
Height_vel <- as.numeric(as.character(DATANY$V16));
#nydata2 <- as.numeric(as.character(DATANY$V9)) /100

par(mfrow = c(1,1))
{
plot(Time_s,Pitch_ref,type="l",col=c("red"));
lines(Time_s,Pitch_mix,type="l",col="blue");
lines(Time_s,Pitch_angle,type="l",col="green");
} #Plot Pitch

{
Pitch_mixN <- scale(Pitch_mix);
Pitch_angleN <- scale(Pitch_angle);
Pitch_refN <- scale(Pitch_ref);
plot(Time_s, Pitch_refN, type="l",col="blue")
lines(Time_s, Pitch_mixN, type="l",col="red")
lines(Time_s, Pitch_angleN, type="l",col="green")
} #Plot Pitch nomralized

{
  plot(Time_s,Roll_ref,type="l",col=c("red"), ylim = c(-8,4))
  lines(Time_s,Roll_mix,type="l",col="blue")
  lines(Time_s,Roll_angle,type="l",col="green")
} #Plot Roll

{
  plot(Time_s,Yaw_ref,type="l",col=c("red"), ylim = c(-4,9))
  lines(Time_s,Yaw_mix,type="l",col="blue")
  lines(Time_s,Yaw_vel,type="l",col="green")
} #Plot Yaw


{
  Yaw_mixN <- scale(Yaw_mix, center=TRUE);
  Yaw_refN <- scale(Yaw_ref);
  Yaw_velN <- scale(Yaw_vel);
  plot(Time_s,Yaw_refN,type="l",col=c("red"), ylim = c(-4,7))
  lines(Time_s,Yaw_mixN,type="l",col="blue")
  lines(Time_s,Yaw_velN,type="l",col="green")
}
