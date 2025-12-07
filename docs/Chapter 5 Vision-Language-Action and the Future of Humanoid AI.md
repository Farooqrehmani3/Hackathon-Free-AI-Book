**Meta Description:**
 Integrate large language models with humanoid robotics to enable natural language control, exploring voice-to-action systems, cognitive planning, and the convergence of conversational AI with physical embodiment.

**Voice-to-Action with OpenAI Whisper**
Natural language interfaces make robots accessible to non-expert users. Students integrate Whisper for speech recognition, converting voice commands into text. The challenge extends beyond transcription—handling ambiguity, accents, and background noise in real environments. Students implement far-field microphone arrays like ReSpeaker, learning audio signal processing techniques that improve recognition accuracy in challenging acoustic conditions.

**Cognitive Planning with Large Language Models**
LLMs can decompose high-level commands into executable robot actions. When told "clean the room," the robot must plan a sequence: navigate to each object, classify whether it belongs there, grasp it, and place it appropriately. Students build systems where LLMs generate action plans in natural language, which a translator converts to ROS 2 action sequences. This teaches the interface between symbolic reasoning and low-level control.

**The Capstone: Autonomous Humanoid Project**
The course culminates in an integrated demonstration combining all learned skills. The humanoid receives a voice command, uses an LLM to plan its approach, employs Nav2 for path planning around obstacles, identifies target objects with computer vision, and executes manipulation. Students work in teams, integrating perception, planning, and control subsystems. This mirrors real robotics engineering where success requires coordinating multiple specialized components.

**Multi-Modal Interaction and Future Directions**
Advanced human-robot interaction combines speech, gesture, and vision. Students explore systems where robots interpret pointing gestures, make eye contact during conversation, and use social cues to predict human intentions. Looking forward, the field moves toward foundation models for robotics—general-purpose policies trained on diverse tasks that can adapt to new situations with minimal fine-tuning. Students discuss implications for deployment, safety, and the societal impact of capable humanoid robots entering human spaces.