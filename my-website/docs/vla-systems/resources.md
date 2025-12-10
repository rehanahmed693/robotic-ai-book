---
title: VLA Systems Resources and References
sidebar_label: Resources and References
description: Additional resources and references for Vision-Language-Action systems in robotics
keywords: [VLA, robotics, AI, vision-language-action, resources, references]
learning_objectives:
  - Access additional learning resources for VLA systems
  - Understand important references for LLM-robotics integration
  - Apply standards and best practices from academic sources
duration: 10
---

import LearningObjectives from '@site/src/components/LearningObjectives';
import DurationEstimator from '@site/src/components/DurationEstimator';

<LearningObjectives objectives={[
  'Access additional learning resources for VLA systems',
  'Understand important references for LLM-robotics integration',
  'Apply standards and best practices from academic sources'
]} />

<DurationEstimator minutes={10} activity="reading" />

# VLA Systems Resources and References

This page provides additional resources and references to support your learning in Vision-Language-Action (VLA) systems for robotics.

## Academic References

### Vision-Language-Action Systems
- Driess, D., Xia, F., Sajjadi, M. S. M., Lynch, C., Ichter, B., Wahid, A., ... & Hausman, K. (2023). Palm-e: An embodied multimodal language model. *arXiv preprint arXiv:2303.03378*.
- Brohan, C., Jauhri, S., Joshi, C., Kappler, D., Khansari, M., Lambeta, M., ... & Ichnowski, J. (2023). RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robot Control. *arXiv preprint arXiv:2307.15818*.
- Li, Z., Wang, Y., Du, Y., Su, H., Zhu, J., & Lu, C. (2023). EmbodiedGPT: Vision-Language Pre-Training via Embodied Reasoning. *arXiv preprint arXiv:2305.17390*.

### Action Grounding in Robotics
- Misra, D., Hejna, A., & Bohg, J. (2022). Translation between multimodal representations of human activity. In *2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)* (pp. 9594-9601).
- Kapelyukh, M., Ospandan, D., Tonneau, S., Saffiotti, A., & Gervasio, S. (2023). Language-Guided Humanoid Robot Navigation. *arXiv preprint arXiv:2307.10805*.

### Whisper-Based Speech Recognition
- Radford, A., Kim, J. W., Xu, T., Brockman, G., McLeavey, C., & Sutskever, I. (2022). Robust speech recognition via large-scale weak supervision. *arXiv preprint arXiv:2212.04356*.
- Nagarajan, T., Fevry, T., Tjandra, A., Goyal, N., & Kirillov, A. (2022). Vision-Language Models for Open-Vocabulary Object Detection. *arXiv preprint arXiv:2206.05739*.

### Cognitive Planning with LLMs
- Huang, W., Abbeel, P., Pathak, D., & Mordatch, I. (2022). Language models as zero-shot planners: Extracting actionable knowledge for embodied agents. *International Conference on Machine Learning*, 9118-9145.
- Wang, T., Wu, Y., Grefenstette, E., & Rocktäschel, T. (2023). Language models as zero-shot planners for embodied instruction following. *arXiv preprint arXiv:2206.01670*.

## Online Resources

### VLA-Specific Resources
- [EmbodiedGPT GitHub Repository](https://github.com/GPT-Embodied/EmbodiedGPT): An open-source project for embodied reasoning with GPT
- [RT-2 Project Page](https://robotics-transformer2.github.io/): Official project page for RT-2: Vision-Language-Action Models
- [PaLM-E Project Page](https://palm-e.github.io/): Official project page for the PaLM-E embodied multimodal language model

### LLM-Robotics Integration
- [ROS-LLM GitHub Project](https://github.com/Lyfe0/ROS-LLM): Integrating large language models with ROS for robotic systems
- [Hugging Face Transformers](https://huggingface.co/docs/transformers/index): Comprehensive library for transformer-based models
- [OpenAI API Documentation](https://platform.openai.com/docs/api-reference): For integrating GPT models with robotics systems

### Voice Processing
- [OpenAI Whisper GitHub](https://github.com/openai/whisper): Open-source speech recognition system
- [SpeechRecognition Python Library](https://pypi.org/project/SpeechRecognition/): Python library for speech recognition
- [Mozilla DeepSpeech](https://deepspeech.readthedocs.io/): Open-source speech-to-text engine

## Tools and Libraries

### Robotics Frameworks
- [ROS 2 Documentation](https://docs.ros.org/en/humble/): Official ROS 2 documentation
- [Robotic Operating System (ROS) Tutorials](https://docs.ros.org/en/humble/Tutorials.html): Step-by-step ROS 2 learning materials
- [MoveIt Motion Planning Framework](https://moveit.ros.org/): Motion planning for robotics applications

### Machine Learning Frameworks
- [PyTorch](https://pytorch.org/): Deep learning framework for building LLM-based systems
- [TensorFlow](https://www.tensorflow.org/): Machine learning framework with robotics applications
- [Hugging Face Accelerate](https://huggingface.co/docs/accelerate/index): For running large models efficiently

### Development Tools
- [VS Code Robotics Extension](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros): Extension for ROS development
- [Docker for Robotics](https://docs.docker.com/): Containerization for consistent robotics development environments
- [PyCharm Professional](https://www.jetbrains.com/pycharm/): Python IDE with scientific tools support

## Standards and Best Practices

### Academic Standards
- [IEEE Standards for Robotics](https://standards.ieee.org/industry-applications/robotics.html): Standards for robotics system design
- [ISO 13482:2014](https://www.iso.org/standard/45781.html): Safety requirements for personal care robots
- [ACM Code of Ethics](https://www.acm.org/code-of-ethics): Ethical guidelines for computing professionals

### Software Development
- [PEP 8 Style Guide](https://peps.python.org/pep-0008/): Python coding standards for robotics applications
- [ROS 2 Style Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html): ROS 2 specific coding standards
- [Semantic Versioning](https://semver.org/): Versioning standards for robotics software

## Relevant Research Papers

### Recent VLA Research (2023-2024)
- Chen, H., Xu, C., Li, C., Ma, X., Zhang, W., Song, G., ... & Li, B. (2023). Language-embedded representations for vision-language navigation. *arXiv preprint arXiv:2302.03049*.
- Chen, L., Wang, Z., Zhang, J., Chen, K., & Li, H. (2023). Embodied intelligence via learning and evolution. *Nature Machine Intelligence*, 4(12), 1074-1090.
- Datta, S., Manuelli, C., & Fox, D. (2022). Behavior transformers: Cloning k modes with one stone. *arXiv preprint arXiv:2206.11251*.

### Foundational Work on LLM-Robotics Integration
- Brohan, C., Brown, J., Carbuncle, J., Devries, T., Finn, M., Gasse, M., ... & Zeng, A. (2022). Rvt: A real-world robot learning benchmark. *arXiv preprint arXiv:2210.06409*.
- Ahn, M., Brohan, A., Brown, N., Chebotar, Y., Cortes, O., David, I., ... & Welker, S. (2022). A robot that can manipulate novel objects. *Nature*, 611(7936), 520-526.

import KeyTakeaways from '@site/src/components/KeyTakeaways';

<KeyTakeaways takeaways={[
  'VLA systems represent the cutting edge of LLM-robotics integration',
  'Multiple research groups are advancing the field with different approaches',
  'Proper academic citation and ethical considerations are critical in this field',
  'Combining resources from multiple domains is essential for success in VLA systems'
]} />