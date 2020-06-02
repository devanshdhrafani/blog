---
title: "SLAM and Autonomous Navigation of Differential Drive robot"
date: 2020-05-31
tags: 
  - slam
  - ROS
  - autonomous navigation
  - mobile robot
categories:
  - Project
  - Tutorials
#header:
 # image: "/assets/images/diffdrive/header.png"
excerpt: "ROS package that implements SLAM and Autonomous Navigation on a custom 2 wheeled Differential Drive robot in Gazebo"
classes: wide
---

Software simulation in ROS can help you learn about how to make robots "think." It is a long way to go from a line following bot to a self-driving car!  This makes software development one of the hardest fields to explore for enthusiasts. This is where the Robot Operating System (ROS) comes in. The highly modular nature of ROS facilitates a developer to focus on his domain and use one of the many open-source ROS packages to fill in the gaps.
{: .text-justify}

In this project, I developed a ROS package that implements SLAM and Autonomous Navigation on a custom 2 wheeled Differential Drive robot in Gazebo. Throughout the development of this project, I learnt several new ROS concepts which are essential to understand for any beginner. In this post, I will summarise the steps that I followed (along with relevant links) and hopefully, in the end, you will learn something new. 
{: .text-justify}

Bur first, let's watch the final result:
{% include video id="jbd2p1llsqA" provider="youtube" %}

You can find the code [here](https://github.com/devanshdhrafani/diff_drive_bot){: target="_blank"}.

## URDF and Sensors

