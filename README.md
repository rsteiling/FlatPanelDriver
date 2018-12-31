# Arduino-based Flat Panel Driver
This project provides Arduino-based controller functionality to drive and 
interface to an LED panel intended to take flat-field images for 
astrophotography.  The implementation permits both manual (knob-settable)
driving intensity, as well as compatibility with the Alnitak Flat Panel
interface which is supported by a number of AP sequencing programs.

## Project Contents

* Arduino source code
* Alnitak emulation source to test the Arduino implementation
* Electrical schematic against which this was developed
* Relevant Datasheets

## Theory of Operation

### Background and Goals

A flat panel is used for various photography applications, with this specific 
case geared toward astrophotography.  The flat panel allows characterizing
vignetting and other aberrations across an optical train by providing even
illumination across the optical element.  The accuracy of such a
characterization depends heavily on the true "flatness" of the illumination used
to gather frames to establish this characterization.

This project aims to design an illuminated element suitable for this
characteriztion with a focus on (a) flexibility and compatibility with
astrophotography setups and software, and (b) an electrical and mechanical 
design that provides adequate flatness.

### Design Decisions

Electrically, this project assumes driving a collection of LEDs which together
can be used to establish the "backlight" for the flat panel.  The LEDs selected
in this project are 6000K (roughly daylight) to provide good character across
all astrophotography filters, though other LED varieties could be swapped in
with appropriate schematic changes.

Because manufacturing tolerances in LEDs, resistors, and other components can
produce variance in effective LED brightness, it is critical to design a system
that promotes the greatest guarantee of even illumination.  Simply driving a
number of LED strands across a resistor is flatly insufficient (pun intended).
Instead, this design uses constant-current LED drivers to guarantee the current
passing through each strand.  This eliminates most manufacturing tolerances that
would otherwise affect the panel performance.

Because such LED drivers require some type of serial communication interface,
and because we also want to provide communicative compatibilty for
astrophotography sequence programs (i.e. via the Alnitak protocol), the design
selects an Arduino Uno to provide these capabilities.  Any number of small board
computers could be used, but the ubiquity of the Uno makes it an easy choice
here.

## Astrophotography Flat Panel Alternatives

This is certainly not the most economical panel nor does it provide the 
quickest way to construct your own.  But, it's fun, it's hands-on, it's 
from-scratch, and (in my opinion) it provides an excellent solution with great 
versatility, expandability, and flat illumination accuracy.

## TODO

This project is should be considered an alpha release until otherwise stated
here.  That said, the master branch is always tested on a hardware setup
consistent with the schematic and verified, so it can be considered working.
There are areas of cleanup that are needed yet, and probably improvements in the
design (when is this not true?), but as always, you implement this project at 
your own discretion and risk.

## Feedback

Please drop me a line if you enjoy or hate this project.  I've provided both 
types of feedback to myself already.
