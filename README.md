# Transport_bacterium by 50 particles
2d particle simulation for synthesize bacterium flock behaviour

## Four different loads to be transported
<p float="left">
  <img src="https://github.com/wawachen/Transport_bacterium/blob/master/bin/circle.png" width="200" />
  <img src="https://github.com/wawachen/Transport_bacterium/blob/master/bin/square.png" width="200" /> 
  <img src="https://github.com/wawachen/Transport_bacterium/blob/master/bin/peanut.png" width="200" />
</p>

<p float="left">
  <img src="https://github.com/wawachen/Transport_bacterium/blob/master/bin/circle1.png" width="200" />
  <img src="https://github.com/wawachen/Transport_bacterium/blob/master/bin/square1.png" width="200" /> 
  <img src="https://github.com/wawachen/Transport_bacterium/blob/master/bin/peanut1.png" width="200" />
</p>

Our particles has the perception of load height distribution, and it can push the load by generating the force along the load centre.

## Results
<p float="left">
<img src="https://github.com/wawachen/Transport_bacterium/blob/master/ezgif.com-gif-maker%20(1).gif" width="250" height="250"/>
<img src="https://github.com/wawachen/Transport_bacterium/blob/master/ezgif.com-gif-maker%20(2).gif" width="250" height="250"/>
</p>
<p float="left">
<img src="https://github.com/wawachen/Transport_bacterium/blob/master/ezgif.com-gif-maker%20(3).gif" width="250" height="250"/>
<img src="https://github.com/wawachen/Transport_bacterium/blob/master/ezgif.com-gif-maker.gif" width="250" height="250"/>
</p>

## How to use it?
```
python bin/Bacterium_drones.py --scenario drone_baterium1.py
```
To change the load type, change the shape in Bacterium_drones.py 
```
shape = "circle_nonuni" or "square_uni" or "peanut_uni" or "U_uni"
```
Currently, you will only get the coverage results. </br>
To enable transport part, please uncomment the code from Line 39 to Line 47
