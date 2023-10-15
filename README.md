# CS-350-Portfolio

# Summarize the project and what problem it was solving
The goal of this project was to design and implement a smart thermostat on a CC3220SF-LAUNCHXL board from TI.  The client wanted an MVP to determine the viability of the project and to decide if it was an industry that they wanted to expand into.  The MVP that was delivered simulated the response from the furnace using LEDs on the board.  It also demonstrated the network communication capabilities of the board as well as the I2C interface on the board.

# What did you do particularly well?
I did a good job of creating state machines and implementing them in a fashion that was easily readable and organized.  I did this by ensuring that my state diagrams were thorough and clearly stated the transitions between states and by modularizing the code to separate the logic and the functionality of the state machines created.

# Where could you improve?
One area that I could improve is with variable typing and memory management.  When designing the project, I was not concerned about memory or flash usage because of the size of the project, because of this some of my variables had types that were larger than required and the code did not ensure that variables that were not needed anymore were removed from memory.

# What tools and/or resources are you adding to your support network?
One tool that I am adding to my support network is bit level math.  It had been a while since I had worked with hex directly and I was able to re-gain familiarity with bit-wise operations and how they function to manipulate hex values more efficiently.

# What skills from this project will be particularly transferable to other projects and/or coursework?
One skill that I practiced a lot throughout the course was reading documentation.  I have read a lot of documentation in the past, but to get an understanding of how to implement the interfaces used for this project there was a lot of reading required and a lot of edge cases to consider.  The documentation for embedded boards is also very thorough and long so I was able to get a lot of practice finding the information I needed in a document that was 600+ pages long.

# How did you make this project maintainable, readable, and adaptable?
To keep this project maintainable, I modularized the code as much as possible.  Instead of having the logic for each feature in a switch statement directly, I placed the logic into individual functions.  This means that when reviewing the code, each state had one function call that was descriptive of what was being done and you could go to that function to investigate the logic associated with that state.  This also means that the system is easy to expand because there are functions created for several of the tasks already.
