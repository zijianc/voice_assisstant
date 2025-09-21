






**GENG4412/5512 Engineering Research Project**


**Final Report Preparation Guide** 

**Semester 2, 2025**










**Associate Professor Jeremy Leggoe**

School of Engineering, University of Western Australia


**Associate Professor Dianne Hesterman**

School of Engineering, University of Western Australia







**© Aug 2025**





# **DECLARATION OF CONTRIBUTION**

You must provide the required information below and sign this declaration page. When you submit your report, you are confirming that the information provided is correct.  See Section 2.3 of this guide for more information.

**My contribution**

Provide details here of your contribution to the project and the contribution of others.




**Use of AI tools**

I have used AI tools in the preparation of my report: Yes/No

Details of how AI tools were used: 




In accordance with University Policy, I certify that:

*The above information is correct, and the attached work submitted for assessment is my own work and that all material drawn from other sources has been fully acknowledged and referenced.*


Student signature   \_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_		Date   \_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_



**Supervisor confirmation**

To the best of my knowledge, the student’s contribution outlined above is correct.



Supervisor signature   \_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_		Date   \_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_



**Project Summary** 

This project designs and implements a speech-enabled assistant for the UWA campus shuttle bus to provide natural, hands-free interaction between passengers and the service. The motivation is to improve accessibility, safety, and overall user experience, particularly for first-time visitors and mobility-impaired passengers, in the challenging acoustic environment of a moving bus. The goal is to deliver a low-latency, robust, and privacy-aware conversational interface that can reliably answer shuttle and campus queries.

The system was developed as a modular pipeline using ROS 2. It combines noise-robust voice activity detection (TEN-VAD), real-time speech-to-text via streaming APIs, a dialogue engine with optional retrieval-augmented access to UWA knowledge, and text-to-speech for clear spoken responses. Wake-word activation and half-duplex design were incorporated to reduce false triggers and suppress echo from onboard speakers. The modular design supports flexible testing and future extensions.

Key results demonstrate that deep-learning based VAD significantly improves speech recognition accuracy in noisy bus conditions, while end-to-end responsiveness meets real-time interaction requirements. Optimizations reduced text-to-speech latency by almost half, and stricter wake-word matching lowered unintended activations. Sentence-level streaming of responses improved the perceived smoothness of conversations.

Future work will focus on extended on-vehicle trials with improved microphone arrays, multilingual and multi-speaker support, tighter integration with shuttle telemetry, and offline/edge model deployment for resilience to connectivity issues. A review of privacy and safety considerations will also be required before production use.

Overall, the project demonstrates a practical and modular pathway to speech-enabled passenger interaction in autonomous shuttles, providing both immediate improvements in accessibility and a solid foundation for future campus transport innovations.
**List of Publications**

If the project has generated manuscripts that have been accepted or submitted to journals or conferences for publication, then a list of these publications should be provided. Only those manuscripts that have already been submitted to a conference or journal should be listed. You do not list any papers that are in the ‘preparation’ phase.  Be sure to state whether a paper has been submitted or has been published.  

For example,

Submitted for publication:

Student, J. Q., Supervisor, J. M, and Otherguy, R. J. (2026, March 3-6). *A novel approach to modelling vortex-induced vibration in underwater pipelines*. 26<sup>th</sup> Australasian Fluid Mechanics Conference, 2026, Sydney, Australia.

**Acknowledgements**

On this page, individuals or organisations that have provided significant assistance to you during the project can be acknowledged. This might include professional support, such as your supervisor or the Engineering Workshop, as well as personal support, such as family.  


**Table of Contents**

Signed Declaration 	i

Project Summary	ii

List of Publications	iii

Acknowledgements	iv

Table of Contents	v

List of Figures	v

List of Tables	v

Nomenclature	vi

1. Purpose of the Report	1

2. Report Structure	2

2.1 	Overview	2

2.2	Title Page	3

2.3	Signed Declaration	3

2.4	Project Summary	5

2.5 	Preamble	5

2.6	Formatting Considerations	6

3.	Introduction	10

3.1 Introduction	10

3.2 Background	10

3.3 Project Objectives	10

3.4 Structure of Report	11

4. Project Process 	12

4.1 Experimental Investigations	12

4.2 Modelling Investigations	12

4.3 Design Investigations	13

4.4 Engineering Practice Investigations	13

4.5 Extended Literature Review	13

5. Results and Discussion	14

6. Conclusions and Future Work	16

References	17 

Appendices	18

Appendix A: Example Title Page	19


**List of Figures**

Figure 2.1: Schematic illustration of a nanoindentation model	7

Figure 2.2:  Extension of Specimen A under axial load	8


**List of Tables**

Table 2.1: Average exposed surface areas of indented reinforcement particles, as determined by analysis of SEM images	8

**Nomenclature**

*A*	Area of a circle

*r*	radius

*v*	linear velocity

*w*	angular velocity


2

**1.	Purpose of the Report**

The purpose of this final report is to communicate the motivations, methods, and outcomes of the *Speech-Enabled Campus Shuttle Bus* project. The report is intended to document both the technical development process and the engineering rationale behind the project, providing a clear record of achievements, limitations, and future directions.

In particular, the report will:

- Present the **objectives** of designing and implementing a speech-enabled assistant for the UWA autonomous shuttle bus, and the motivations for pursuing these objectives in the context of enhancing passenger interaction and accessibility.
- Situate the project within its **engineering context**, highlighting challenges such as noisy environments, latency in speech recognition, and the need for domain-specific knowledge integration.
- Describe the **techniques, tools, and procedures** applied in the project, including modular ROS2 nodes, TEN-VAD for noise reduction, OpenAI speech-to-text APIs, retrieval-augmented generation for knowledge access, and text-to-speech evaluation.
- Report the **findings**, including system-level performance in simulation and early deployment, as well as optimizations undertaken to improve latency, accuracy, and robustness.
- Provide a **critical discussion** of the results relative to the project objectives and prior work, identifying both strengths and areas for improvement.
- Conclude with a set of **recommendations**, outlining next steps for full-scale deployment, further testing, and potential extensions to other intelligent transport systems.

The primary audience for this report consists of academic staff in the School of Engineering, who will evaluate the project as part of the GENG4412/5512 assessment. The report is also intended to serve as a resource for future students and researchers who may extend the work, particularly in the areas of speech-enabled robotics and autonomous vehicle interaction.

This document is designed to stand alone as a comprehensive account of the project, reflecting the expected expertise of students completing the Master of Professional Engineering. It demonstrates not only technical implementation but also engineering judgement, critical evaluation, and professional communication.

**2.	Report Structure**

This report is organised into six main sections, supported by references and appendices:
- Section 1 — Purpose of the Report: Outlines the motivation for producing this final report and its role in communicating the project’s objectives, methods, and findings.
- Section 2 — Report Structure: Provides an overview of how the document is organised to guide the reader.
- Section 3 — Introduction: Presents the problem statement, background context, and project objectives; situates the work within current challenges in speech‑enabled transport systems.
- Section 4 — Project Process: Details the technical development of the speech‑enabled shuttle assistant, focusing on the modular ROS 2–based pipeline, including voice activity detection, speech‑to‑text (STT), optional retrieval‑augmented generation (RAG) knowledge, and text‑to‑speech (TTS) integration. Summarises relevant evaluations of latency, accuracy, and robustness.
- Section 5 — Results and Discussion: Reports key findings such as performance in noisy environments, latency reductions, and passenger interaction benefits, and critically discusses these against the objectives and prior literature.
- Section 6 — Conclusions and Future Work: Summarises contributions and outcomes, identifies limitations, and recommends next steps including deployment trials, scalability, and integration with transport services.
- References and Appendices: Lists cited sources (APA 7) and provides supporting technical material, including the project Gantt chart.


3.	Introduction

This project develops a speech-enabled assistant for the University of Western Australia (UWA) campus shuttle bus to enable natural, hands-free interaction between passengers and the vehicle service. Public transport cabins present a difficult acoustic environment: continuous engine and road noise, transient chatter, and on-board loudspeaker playback all degrade far-field voice capture, inflate wake-word false triggers, and increase end-to-end latency. The system targets a robust and responsive dialogue experience that improves accessibility, reduces cognitive load for new or infrequent riders, and provides accurate, context-aware campus information during travel.
In simple terms, the project aims to make it easier for passengers to talk to the shuttle and get useful answers, even in a noisy moving vehicle.


3.1 	Background

Voice interfaces are increasingly used in vehicles, but performance in shared, noisy settings remains a challenge. Traditional energy-threshold VADs are prone to false positives under varying background noise, while cloud STT/LLM/TTS introduce latency and echo coupling via cabin speakers. To address these issues, this project adopts a modular ROS 2 architecture with two complementary front ends for speech detection and recognition:
•	A lightweight RMS-based VAD (realtime_stt_node.py) with dynamic thresholding, pre-speech buffering, and half-duplex gating linked to TTS playback.
•	A deep-learning TEN-VAD pipeline (ten_vad_stt_node.py) that performs frame-level speech activity inference at 16 kHz with improved accuracy and noise robustness.

Transcription uses OpenAI’s streaming STT models (e.g., gpt-4o-mini-transcribe), gated by strict, sentence-initial wake-word rules (e.g., “Hi/Hey/Hello Captain” and a Chinese wake-word used for demonstration) and an SNR filter to suppress self-echo. Dialogue is handled by a streaming LLM node (llm_node_cn.py) with sentence-level emission, content-noise filtering, and a concise rolling-summary memory. Text-to-speech is provided by OpenAI/Qwen TTS with robust playback and explicit status signalling (topic tts_status) so that STT can pause during audio output and resume with a configurable hangover. The system is packaged for containerised deployment with helper scripts and instrumentation for performance monitoring.
In simpler words, the system combines advanced noise detection, real-time transcription, smart dialogue handling, and clear speech playback to deliver smooth conversations between passengers and the shuttle.

3.2	Project Objectives

The project pursues the following objectives:
•	Robust activation: Achieve reliable wake-word triggering in a moving bus with variable noise, using strict sentence-initial matching and SNR gating to reduce false activations.
•	Low-latency interaction: Maintain responsive end-to-end behaviour through pre-buffering, sentence-level LLM streaming, and optimised TTS playback.
•	Accuracy and relevance: Provide correct, concise answers about shuttle operations and campus facilities; optionally augment with a curated UWA knowledge base (RAG) where appropriate.
•	Natural speech output: Generate clear on-board TTS with buffering to reduce fragmentation and an explicit half-duplex design to suppress self-echo.
•	Engineering quality: Deliver a modular ROS 2 system with clear topics, configuration via environment variables, and diagnostics for reproducible testing and tuning.
•	Deployability: Run reliably inside a container environment with start scripts, device enumeration, and fallbacks for constrained audio setups.
Put simply, the objectives are to make the system accurate, fast, clear, well-engineered, and ready for real-world deployment.

3.3	Scope, Assumptions, and Limitations

•	Scope: The assistant focuses on passenger information and conversational support. It does not issue safety-critical driving commands or interface with vehicle actuation.
•	Connectivity: STT/LLM/TTS use cloud services; performance depends on network quality. Edge/offline models are considered future work.
•	Hardware: Experiments assume a single far-field microphone; array beamforming and diarisation are out of scope but discussed as extensions.
•	Privacy: Only minimal text logs are retained for debugging; sensitive audio retention is avoided in normal operation.
In other words, the project delivers a useful passenger assistant, but it is not responsible for controlling the bus and still depends on cloud services and basic hardware setups.
3.4 Structure of the Report

Section 4 details the system design and implementation, including VAD, STT, LLM, and TTS integration with half-duplex gating and performance instrumentation. Section 5 presents results and discussion, including latency observations, wake-word precision, and robustness in noisy conditions. Section 6 concludes with limitations and future work such as on-vehicle trials with array microphones, intent-aware barge-in, telemetry integration (ETA/occupancy), and edge deployment options.
This means the report flows from system design, to results, to conclusions and recommendations for future work.



**4.	Project Process** 

Choose a name for this section that it is appropriate to your project, e.g., Experimental Method, Model Formulation, Design Approach, Data Collection, or Methodology and Methods.

This section describes the “process” by which the project objectives were achieved. The nature of this process will vary according to the type of project. **The guiding principle for this section is that it should provide all the necessary information for subsequent researchers (students and staff) to be able to repeat the work.**

Elements of this section for different types of investigation could include the following (note that more than one may apply for any given investigation).

Note: All projects must be both relevant and apply to engineering and engineering practice.

4\.1 	Experimental Investigations

For experimental investigations include as a minimum:

- A description of experimental procedures, including references to relevant test standards
- Diagrams and descriptions of experimental apparatus, and their accuracy
- Specimen designs, preparation and/or sources
- A description of any health and safety requirements in the laboratory or the field.  This includes reference to relevant Material Safety Data Sheets (MSDS), if applicable, although it would be usual to relegate the data sheets to an appendix, and
- A description of the methods used to calculate key parameters from the raw data

For example, to describe a tensile test a drawing of the specimens showing the specimen dimensions should be provided, along with a description of how the specimens were fabricated and prepared for testing (including any heat treatment), the relevant ASTM standard and section(s), the equipment used, gripping arrangements, loading rate, and the method used to measure displacement and strain, and the measurement accuracy. The method used to calculate the elastic modulus, yield strength. Ultimate tensile strength and/or failure strain from the experimental record should also be described.

4\.2 	Modelling Investigations

Modelling investigations often include model derivation and then implementation using a software package or written code.  For modelling investigations include as a minimum:

- Model derivation (or modification) 
- A description and diagram of the model configuration
- A description of any software tools used and/or created and reason(s) for choice
- Descriptions of the model input options selected (boundary conditions, constitutive behaviours, etc)
- A discussion of the process by which the final model configuration was determined (for example, discussing the tests undertaken to ensure that the model has converged towards the true solution)
- A description of any model validation tests (in some cases, this may appear in the results and discussion section), and
- A discussion of any limitations in the model or software

4\.3 	Design Investigations

For design investigations include as a minimum:

- A description of the constraints imposed on the design (space, particular materials, cost limits, etc)
- A discussion of the criteria that will be used to make design choices and evaluate the success of the final design
- A discussion of the ideation process used to generate alternative designs
- Descriptions of any design tools or procedures employed (such as software) and reason(s) for choice
- A discussion of relevant design codes or standards, including sections, or requirements that the design is informed by or must comply with (this may be included in the criteria or constraints)
- A description of any experiments or tests undertaken to evaluate the final design or product (following the guidelines provided above for experimental procedures), and
- A discussion of any health and safety considerations in the fabrication, testing, and operation of any physical prototype. 

4\.4 	Engineering Practice Investigations

For engineering practice and techno-economic investigations include as a minimum:

- A description of criteria used to choose appropriate methodology (e.g., qualitative versus quantitative, etc.)
- A description of data collection methods (surveys, interviews, literature, focus groups, etc.)
- A description of any required ethics approval and how any constraints were implemented and managed
- A description of the principal information sources, including any relevant codes or standards
- A description of the tools or techniques to be used in formulating any new approaches 
- A discussion of the criteria that will be used to make decisions and evaluate the success of any recommended approach, and
- A discussion of limitations of chosen approach and any safety considerations if applicable to the project.

4\.5 	Extended Literature Review

For projects that are predominantly a literature review, this section should include:

- A description of data collection methods (e.g., databases/indexes and key search terms) 
- A description of the principal information sources and reason(s) for choice
- A discussion of the process used to select and evaluate literature for the review
- Description of primary themes the literature was grouped under and reason(s) for choice.

**5.	Results & Discussion**

**This section is very important.  Make sure the content you write here is clear and defensible.**

This section should provide the results of the investigation, and discuss their implications in the context of the original objectives, the background history, and the pre-existing state of the art. **The guiding principle for this section is that it should describe what has been done and demonstrate how well the findings are understood.**  

For *experimental* and *modelling* studies, the “results” are easy to identify. For *design* studies, the final design, and the expected or measured performance of that design will represent the “results”. For *techno-economic* projects, the “results” will be the outcomes of your investigation. 

For an *extended literature review* project, the findings of the review will form “results”; and the discussion should include critical analysis of the literature and a proposal for future action based on the results of the review; for example, the design of a future experiment or extension of an existing model, key findings or recommendations to inform future work within an existing research project, and/or financial analysis of options suggested by the literature review.

Depending on the volume of raw data, it may not be appropriate to present all raw data in the body of the report – it is usual to only provide characteristic elements of the raw data and key results in the main body of the report, and to provide the remainder in an appendix. The results in the main body of the report should present all relevant “analysed” data, in graphical or tabular form (as appropriate). In many instances, it will be appropriate to present “comparative” results – these may be results comparing the behaviour under different sets of conditions, or comparisons of performance under the recommended new approach with former approaches, for example. As much as possible, a format should be selected that permits the results being compared to appear on a single page – it is very difficult for the reader to compare images on multiple pages, or to reconcile tables on different pages.

Important figures, especially graphs, must be presented in the main text if they are to be discussed. For example, if ten stress-strain plots are available, a single representative, composite, or average plot could be presented in the main text, with the remaining individual plots relegated to the appendix. Think carefully about what to include in the main body of the report and what could be included in the appendices. There is nothing a reader dislikes more than having to hunt in the appendix for figures that illustrate the discussion. 

The purpose of the discussion is to place the results in context and demonstrate your understanding of the implications (and limitations) of the results. The following elements may be present, depending on the nature of the project:

Compare the findings with any original expectations:

- How did the original hypothesis hold up?
- How has the hypothesis evolved based on the results?
- Does the proposed design meet the original objectives?

Compare the findings with the pre-existing state of the art:

- How do the results compare with the results of previous investigations?
- How has the investigation added to the body of knowledge in the field?
- How does the new design or procedure improve on existing approaches?

Compare the proposed approach with alternatives:

- For design and engineering practice investigations, it is important to discuss the alternative approaches that were considered, and the reasons for selecting the final approach.
- Were other modelling tools available? Why were they not adopted? Do they offer any possible advantages for future investigations?

Discuss the limitations of the current project:

- Do the current experimental rigs or software impose limits on the extent to which the objectives could be explored or on the application of findings?
- What could be done better? Which assumptions should be challenged?
- What does the new design fail to achieve?
- Which aspects of practice were not able to be explored properly?
- Describe any remaining issues.

It is important that limitations be discussed openly and not be regarded as negative – it is very important to understand the limitations of a technique to properly interpret and apply results. In many cases, a recognition of the limitations demonstrates an improved understanding as an outcome of the investigation. 

Similarly, it is important that the discussion address any arbitrary choices made during the project. In most projects, it will be necessary to take some arbitrary choices to make progress. These choices should be identified, and alternative approaches should be considered in the discussion. Once again, this should not be viewed negatively – demonstrating an awareness of such choices is a way of displaying the depth to which an issue has been understood and may provide motivation or directions for future studies.

**Remember – all statements and arguments in the report must be supported, either by results, or information available in the literature. This is especially important in the Discussion section. Avoid empty “hand waving”!**

**6.	Conclusions & Future Work**

Conclusions should state concisely the most important findings of the project. This will include:

- A summary of the key arguments / findings developed in the discussion
- An assessment of whether the project objectives have been achieved (and if not, why not), and
- A description of future work arising from the project (e.g., to address unresolved issues, extend the work, implement findings, etc.)

Future Work is an important element of this section.  It allows you to demonstrate greater insight into your project and outcomes, and to make recommendations for the next steps. 

While the conclusions section will generally be brief, care should be taken in writing this section. Remember, it will be the last thing that the audience reads, so it will be the last thing on their mind before they decide on the future of the project (in professional practice), or on the mark (right now)!

**References** 

The reference list is an important “road map” for anyone following up on the project and may prove to be extremely useful if you continue with similar work, either in your professional career or in postgraduate research.  An example of the APA 7th edition referencing system has been provided in this guide.

University Library. (2025). *Library Guides*. University of Western Australia. Retrieved July 29, 2025, from <https://guides.library.uwa.edu.au>. 

**Appendices** 

Appendices may be provided as appropriate, especially to provide original data that do not necessarily fit conveniently into the main text. Material typically included in appendices may include:

- Raw experimental data
- Contour plots from modelling software
- Raw images (i.e., images prior to processing or filtering)
- Software (codes) used to compute results
- Results from previous studies

Appendices are usually named Appendix A, Appendix B, and so forth and provided in the order they are referred to in the main body of the report, e.g. the first appendix referred to in the report is named Appendix A and this is provided first in the appendices.  

It is worth reiterating that important figures, especially graphs, must be placed in the main text if they are to be important subjects in the discussion. 

**Literature Review**

Your full literature review should be included as one of your appendices, unless your project is an extended literature review.  Provide any key points from our literature review in Section 3.2.

` `**Appendix A: Sample Title Page for GENG4412**


**GENG4412 Engineering Research Project Part 2**

**Final Report**

**Project Title Here**





**Jamie Q. Student**

Student Number

School of Engineering, University of Western Australia

**Supervisor: Jessica M. Supervisor**

School of Engineering, University of Western Australia

**Co-Supervisor: Robert J. Otherguy**

Other School or Company Inc**.**



*Word count: 5,435*



**School of Engineering**

**University of Western Australia**


Submitted: 17 October 2025



**Appendix A: Sample Title Page for GENG5512**


**GENG5512 MPE Engineering Research Project Part 2**

**Final Report**

**Project Title Here**





**Cecelia Tzaziki**

Student Number

School of Engineering, University of Western Australia

**Supervisor: Jackie Chan**

School of Engineering, University of Western Australia

**Co-Supervisor: Nicole Mehta**

Other School or Company Inc**.**



*Word count: 6,803*



**School of Engineering**

**University of Western Australia**


Submitted: 13 October 2025

2

