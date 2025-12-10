---
sidebar_position: 1
---

# Educational Component Examples

This document showcases the custom educational components available in this learning platform.

## Learning Objectives Component

The `LearningObjectives` component allows us to clearly define what students will learn in a lesson:

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives objectives={[
  'Understand how to use custom components in educational content',
  'Identify the benefits of structured learning materials',
  'Apply best practices to create effective lessons'
]} />

## Duration Estimator Component

The `DurationEstimator` component helps set expectations for how long an activity will take:

import DurationEstimator from '@site/src/components/DurationEstimator';

<DurationEstimator minutes={15} activity="reading" />

## Key Takeaways Component

The `KeyTakeaways` component highlights the most important concepts that students should remember:

import KeyTakeaways from '@site/src/components/KeyTakeaways';

<KeyTakeaways takeaways={[
  'Custom components enhance the learning experience',
  'Structured content improves comprehension and retention',
  'Visual indicators help focus attention on important information'
]} />

## Assessment Component

import Assessment from '@site/src/components/Assessment';

<Assessment
  question="What is the primary benefit of using structured learning components?"
  options={[
    "Improved visual design",
    "Better comprehension and retention",
    "Faster loading times",
    "More complex interactions"
  ]}
  answer={1}
  explanation="Structured learning components help improve comprehension and retention by organizing information in consistent, predictable ways."
/>

## Code Example Component

import CodeExample from '@site/src/components/CodeExample';

<CodeExample
  code={`// Example of a ROS 2 publisher in Python
import rclpy
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.create_node('publisher_node')
    publisher = node.create_publisher(String, 'topic', 10)
    msg = String()
    msg.data = 'Hello, ROS 2!'
    publisher.publish(msg)`}
  explanation="This Python code demonstrates how to create a basic ROS 2 publisher that sends a string message to a topic."
  language="python"
/>

## Media Display Component

import MediaDisplay from '@site/src/components/MediaDisplay';

<MediaDisplay
  type="image"
  src="/img/docusaurus-social-card.jpg"
  caption="Docusaurus sample image"
  alt="Sample image for demonstration"
/>

This example shows how the educational components work together to create an engaging learning experience. Each component serves a specific purpose in helping students understand and retain the material.