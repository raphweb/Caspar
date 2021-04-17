close all
clear all
clc


angle  = 0:1:359;
speed = 1;

%% Motor 1
fig = figure;
set(fig, 'Name', 'Motor 1');
set(fig, 'Position', [24   562   560   420]);
plot(angle, sign(sin(deg2rad(angle + 90))))
hold on
plot(angle, sin(deg2rad(angle + 90))*speed)
hold off


%% Motor 1
fig = figure;
set(fig, 'Name', 'Motor 2');
set(fig, 'Position', [587   561   560   420]);
plot(angle, sign(sin(deg2rad(angle))))
hold on
plot(angle, sin(deg2rad(angle))*speed)
hold off

%% Motor 1
fig = figure;
set(fig, 'Name', 'Motor 3');
set(fig, 'Position', [26    57   560   420]);
plot(angle, sign(sin(deg2rad(angle + 90))))
hold on
plot(angle, sin(deg2rad(angle + 90))*speed)
hold off


%% Motor 1
fig = figure;
set(fig, 'Name', 'Motor 4');
set(fig, 'Position', [589    55   560   420]);
plot(angle, sign(sin(deg2rad(angle))))
hold on
plot(angle, sin(deg2rad(angle))*speed)
hold off

