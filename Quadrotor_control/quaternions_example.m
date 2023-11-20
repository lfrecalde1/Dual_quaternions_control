clc, close all, clear all;

q = [0.9071 0.3 0 0];
axang = quat2axang(q)
[angle, axis]  = quaternionToAxisAngle(q)