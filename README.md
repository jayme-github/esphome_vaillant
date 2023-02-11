# Control Vaillant heater via esphome
My flat is equipped with a Vaillant VCW 204/3-E-HL gas boiler without any room temperature controller and a very power hungry circulator pump (~90W). I tried to bring down gas and power usage by controlling the heater via [Home-Assistant](https://www.home-assistant.io/) / [esphome](https://esphome.io/) according to my needs.

This works by connecting the Vaillant X6 (debug) interface to an ESP32 to provide sensor data on various temperatures and states as well as controlling the output voltage to the Vaillant 7-8-9 interface to control the supply/flow temperature.

It might also be possible to set the supply/flow temperature (and other parameters) directly via X6 but I did not try as using the 7-8-9 interface (which is designed for exactly that) seemed more safe to me.

To great extend this is based on work by others:
* PCB design for the Vaillant 7-8-9 interface: https://github.com/lal12/vaillant-heater-control-esp
* Vaillant X6 interface: https://github.com/martin3000/ESPhome
* Details on the Vaillant X6 protocol: https://old.ethersex.de/index.php/Vaillant_X6_Schnittstelle
* Lots of discussion around this topic (in German): https://www.mikrocontroller.net/topic/126250?page=single

