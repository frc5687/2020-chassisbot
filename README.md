FRC 5687 The Outliers 2019 Chassisbot
===

This project is a very basic Java implementation for a drivetrain-only FRC robot.  It is intended as a starting point for new members of our programming team, but we also encourage new programmers on any team to take advantage of it!

Please note that while much of our project and code structure follows typical FRC patterns, we do have our own approaches to many aspects.  We make every effort to highlight those differences in our comments and in this readme.  We also use a slightly non-standard set of tools, which are outlined below.

Tools
===
We use [Jaci](https://github.com/JacisNonsense)'s excellent gradlerio toolset in combination with JetBrains' IntelliJ, Atlassian's Sourcetree, and (obviously) GitHub to manage our codebase.  There are free downloads of each of the tools we use, although some do require registration.

Standard FRC Java downloads:
---
- [WPILib](https://github.com/wpilibsuite/allwpilib/releases/tag/v2019.4.1)

Primary tools we use:
---
- [IntelliJ](https://www.jetbrains.com/idea/download/) - our primary IDE (used in place of Eclipse / VS Code)  
- [Git](https://git-scm.com/downloads) - Source-code manager  
- [Sourcetree](https://www.sourcetreeapp.com/) - GUI for git.  This is optional, but all of our explanations will use it.  
- [TortoiseMerge](https://tortoisesvn.net/downloads.html) - GUI for resolving merge conflicts when different programmers make changes to the same file.  Note that this is installed as part of TortoiseSVN, but we only use the TortoiseMerge component.  

Additional tools:  
---
- [WinSCP](https://tortoisesvn.net/downloads.html) - used for remote file access to the roborio (and Raspberry Pi if you use one)  
- [PuTTY](https://www.putty.org/) - used for remote terminal access to the roborio (and Raspberry Pi if you use one)  

Setup
===

Tool installation
---
1. Dowload all the above tools
2. Install WPILib first by extracting the installer from the downloaded archive and running it.  You can uncheck Visual Studio Code and C++ Compiler, but be sure that Gradle, Java JDK/JRE, Tools and Utilities, and WPILib Dependencies are checked.
3. (Optional but recommended) Set your JAVA_HOME environment variable to the JDK installed by the WPILib installer (c:\users\pulbic\frc2019\jdk)
4. Install IntelliJ, git, Sourcetree, and TortoiseMerge by running their installers.
5. (Optional) Install WinSCP and PuTTY.

Workspace setup
---
1. Fork this project (not necessary for 5687 team members)  
 - Simply click the FOrk button above and to the right
2. Checkout the project locally  
 - Open SourceTree
 - Open the "clone" dialog
 - Paste the URL of your github project (eg, https://github.com/[yourgithubidhere]/2019-chassisbot or for 5687 team members, https://github.com/frc5687/2019-chassisbot)
 - Enter your working folder under "Destination Path"
 - Click Clone
3. Configure the project for your team
 - Navigate to your new working folder in windows explorer
 - Edit the build.gradle file and replace 5687 with your team number
 - In the .wpilib folder, edit the wpilib_preferences.json file and again replace 5687 with your team number 
4. Prep the project using gradlew
 - Open a command prompt in your new working folder
 - run "gradlew idea" to create your intellij project
5. Open the project in IntelliJ
 - You can do this by simply double-clicking on the 2019-chassisbot.ipr file in your working folder
