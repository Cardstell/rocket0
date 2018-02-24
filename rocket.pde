import processing.serial.*;

final int rx = 1024, ry = 768-20;
//final int rx = 1920 - 52, ry = 1080-20;
final int countData = (int)(0.4485 * rx);
float angleX = 0.0, angleY = 0.0;
float alt = 0.0, hspeed = 0.0, vspeed = 0.0, acc = 0.0, lat = 50.025806, lon = 78.995038, temp = 22.8;
Serial port;

float[] accData1 = new float[500], accData2 = new float[500];
int accOffset = 0;
float accMax = 0, accMin = 0;
ArrayList<Integer> pointsX = new ArrayList<Integer>();
ArrayList<Integer> timesX = new ArrayList<Integer>();

long timeOffset;

void setup() {
  fullScreen(P2D,2);
  background(0, 0, 20);
  port = new Serial(this, "/dev/ttyUSB0", 9600);
  for (int i = 0;i<500;++i) {
    accData1[i] = 0.0;
    accData2[i] = 0.0;
  }
  pointsX.add(0);
  timesX.add(0);
  timeOffset = System.nanoTime();
}

void updateLimits() {
  float prev;
  if (accOffset>=500) {
    prev = accData1[accOffset-500];
    accData2[accOffset-500] = acc;
  }
  else {
    prev = accData2[accOffset];
    accData1[accOffset] = acc;
  }
  if (acc>accMax)
    accMax = acc;
  if (acc<accMin)
    accMin = acc;
  if (prev >= accMax || prev <= accMin) {
    float mn = 1e9, mx = -1e9;
    for (int i = 0;i<500;++i) {
      int cur = accOffset - i;
      if (cur<0) cur += 1000;
      if (cur>=500) {
        if (accData2[i]==0.0) continue;
        mn = min(mn, accData2[i]);
        mx = max(mx, accData2[i]);
      }
      else {
        if (accData1[i]==0.0) continue;
        mn = min(mn, accData1[i]);
        mx = max(mx, accData1[i]);
      }
    }
    accMin = mn;
    accMax = mx;
  }
  println(timesX.size());
  if (((System.nanoTime() - timeOffset)/1000000000 - timesX.get(timesX.size()-1))>=10) {
    timesX.add(timesX.get(timesX.size()-1)+10);
    pointsX.add(accOffset);
  }
}

void draw() {
  background(0, 0, 20);
  fill(255,255,255);
  stroke(255,255,255);
  //line(rx*0.4, 0, rx*0.4, ry);
  //line(rx*0.4, ry*0.5, rx, ry*0.5);
  //Getting data
  while (port.available() > 0) {
    String buff = port.readString();
    if (buff==null) continue;
    String sbuff[] = buff.split("\\s+");
    if (sbuff.length!=4) continue;
    if (!sbuff[0].startsWith("$>")) continue;
    acc = Float.parseFloat(sbuff[1]);
    angleX = Float.parseFloat(sbuff[2]);
    angleY = Float.parseFloat(sbuff[3]);
    updateLimits();
    accOffset++;
    if (accOffset==1000) accOffset = 0;
  }
  //Text
  textSize(32);
  text("Altitude:", 10, 40);
  text("Vert speed:", 10, 80);
  text("Hor speed:", 10, 120);
  text("Acceleration:", 10, 160);
  text("Lat:", 10, 200);
  text("Lon:", 10, 240);
  text("Temperature:", 10, 280);
  text(String.format("%.0fm", alt), 250, 40);
  text(String.format("%.1fm/s", vspeed), 250, 80);
  text(String.format("%.1fm/s", hspeed), 250, 120);
  text(String.format("%.2fm/s²", acc), 250, 160);
  text(String.format("%.6f°", lat), 250, 200);
  text(String.format("%.6f°", lon), 250, 240);
  text(String.format("%.1f°", temp), 250, 280);
  //Graph
  line(rx, 0, rx, ry);
  line(0, ry, rx, ry);
  line(500, 70, 500, 270);
  line(500, 70, 1000, 70);
  line(500, 270, 1000, 270);
  line(1000, 70, 1000, 270);
  noFill();
  stroke(230, 0, 0);
  int prevX = 1000, prevY;
  accOffset--; if (accOffset<0) accOffset += 1000;
  if (accOffset>=500) prevY = 260 - (int)((accData2[accOffset-500]-accMin)/(accMax-accMin)*180);
  else prevY = 260 -(int)((accData1[accOffset]-accMin)/(accMax-accMin)*180);
  for (int i = 1;i<500;++i) {
    int curX = 1000-i;
    int curY, cur = accOffset - i;
    if (cur<0) cur+= 1000;
    float curdata;
    if (cur>=500) curdata = accData2[cur-500];
    else curdata = accData1[cur];
    curY = 260 - (int)((curdata-accMin)/(accMax-accMin)*180);
    if (curdata!=0.0) line(curX, curY, prevX, prevY);
    prevX = curX;
    prevY = curY;
  }
  accOffset++; if (accOffset==1000) accOffset = 0;
  textSize(12);
  stroke(255,255,255);
  float pointsY[] = new float[5];
  String prec;
  if (accMax-accMin<0.3) {
    pointsY[0] = floor((accMax+(accMax-accMin)/18.0)*100)/100.0;
    pointsY[4] = ceil((accMin-(accMax-accMin)/18.0)*100)/100.0;
    prec = "%.3f";
  }
  else if (accMax-accMin<3) {
    pointsY[0] = floor((accMax+(accMax-accMin)/18.0)*10)/10.0;
    pointsY[4] = ceil((accMin-(accMax-accMin)/18.0)*10)/10.0;
    prec = "%.2f";
  }
  else if (accMax-accMin<30) {
    pointsY[0] = floor((accMax+(accMax-accMin)/18.0));
    pointsY[4] = ceil((accMin-(accMax-accMin)/18.0));
    prec = "%.1f";
  }
  else {
    pointsY[0] = floor((accMax+(accMax-accMin)/18.0)*0.1)*10.0;
    pointsY[4] = ceil((accMin-(accMax-accMin)/18.0)*0.1)*10.0;
    prec = "%.0f";
  }
  pointsY[2] = (pointsY[0] + pointsY[4])/2;
  pointsY[1] = (pointsY[0] + pointsY[2])/2;
  pointsY[3] = (pointsY[2] + pointsY[4])/2;
  textAlign(RIGHT, CENTER);
  for (int i = 0;i<5;++i) {
    int _y = min(max(260 - (int)((pointsY[i]-accMin)/(accMax-accMin)*180),70),270);
    line(500, _y, 495, _y);
    text(String.format(prec, pointsY[i]), 493, _y);
  }
  textAlign(CENTER, TOP);
  for (int i = timesX.size()-1;i>=0;--i) {
    int minOffset = accOffset - 500;
    print(minOffset);
    print(" ");
    print(accOffset);
    print(" ");
    println(pointsX.get(i));
    if ((minOffset>=0 && (pointsX.get(i)>=minOffset && pointsX.get(i)<=accOffset)) || 
      (minOffset<0 && (pointsX.get(i)<=accOffset || pointsX.get(i)>=(minOffset + 1000)))) {
      int _x = accOffset-pointsX.get(i); if (_x<0) _x += 1000;
      line(1000-_x, 270,1000-_x, 275);
      text(timesX.get(i), 1000-_x,277);
    }
    else {
      timesX.remove(0);
      pointsX.remove(0);
      timesX.trimToSize();
      pointsX.trimToSize();
    }
  }
  textAlign(LEFT, BOTTOM);
  //Angle
  noFill();
  stroke(255,255,255);
  ellipse(rx*0.7, ry*0.75, ry*0.45, ry*0.45);
  ellipse(rx*0.7, ry*0.75, ry*0.3, ry*0.3);
  ellipse(rx*0.7, ry*0.75, ry*0.15, ry*0.15);
  line(rx*0.7-ry*0.225, ry*0.75, rx*0.7+ry*0.225, ry*0.75);
  line(rx*0.7, ry*0.75 + ry*0.225, rx*0.7, ry*0.75 - ry*0.225);
  float angLimit = ceil(max(sqrt(pow(angleX,2) + pow(angleY,2)), 30.0)/3)*3;
  textSize(12);
  text((int)(angLimit/3.0+0.01),rx*0.7+ry*0.075 + 3, ry*0.75 - 3);
  text((int)(angLimit/3.0+0.01),rx*0.7 + 3, ry*0.75-ry*0.075- 3);
  text((int)(angLimit/1.5+0.01),rx*0.7+ry*0.15 + 3, ry*0.75 - 3);
  text((int)(angLimit/1.5+0.01),rx*0.7 + 3, ry*0.75-ry*0.15 - 3);
  text((int)(angLimit+0.01),rx*0.7+ry*0.225 + 3, ry*0.75 - 3);
  text((int)(angLimit+0.01),rx*0.7 + 3, ry*0.75-ry*0.225 - 3);
  noStroke();
  fill(230,0,0);
  ellipse(rx*0.7 + angleX/angLimit*ry*0.225, ry*0.75 + angleY/angLimit*ry*0.225, ry*0.03, ry*0.03);
}