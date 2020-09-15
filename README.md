# inverted-pendulum
State feedback based LQR control of an inverted pendulum built from an old Canon printer.

![](img/pend.gif)

Youtube video:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=Zrd7h5_OmEI
" target="_blank"><img src="http://img.youtube.com/vi/Zrd7h5_OmEI/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="200" height="160" border="10" /></a>

# Model
<img src=./img/model.png alt="Model." width="250"/>
<img src=./img/ss.png alt="State Space." width="450"/>

# Microcontroller
Tiva C TM4C123 microcontroller.

<img src=./img/circ.png alt="Circuit." width="500"/>

Results. Stable, somewhat accurate / fast repsonse. Not bad for simulation based approach.

<img src=./img/res.png alt="Results." width="600"/>

# LQR Control
<img src=./img/flow.png alt="Control Flow." width="500"/>

<img src=./img/loop.png alt="Closed Loop." width="300"/>

# Dual PID Controller
A far more practical approach. Stay 'tuned'...
