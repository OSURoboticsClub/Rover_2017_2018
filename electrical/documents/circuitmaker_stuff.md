# How to CircuitMaker for OSURC Mars Rover
----

### Install
Download and install CircuitMaker from [here](https://workspace.circuitmaker.com/Account/SignUp).

### Starting a New Project 
Start a new project and make sure to use the naming convention when making the title, you can't change it later -_-. We use `MR1718 Name`. For example, the motor node would be `MR1718 Motor`. To make things uniform there are generated project images you can download from [here](https://drive.google.com/drive/folders/1vX37lvlG4ao4wY86Rg7hQaVi0l3jGEsV?usp=sharing), so add that on the main project page as well.
Next download the sheets you will need for your project, at the bare minimum you need:

* mcu
* reverse protection
* rs485_tranceiver
* 5v_power
* 3v3_power

Now in CircuitMaker go to `Home -> Project -> Add Existing Document` and browse to where you downloaded those sheets, select them and you are done!

If you ever need to find the actual sheet file that CircuitMaker uses it keeps them in this directory:

`C:\ProgramData\Altium\CircuitMaker {A bunch of garbage here}\Projects\Another string of garbage\More garbage`

There may be multiple directories of `Garbage strings`, so just click around until you find it.

#### Using an OSURC template

Download the sheet template files from this repo and add them to `C:\Users\Public\Documents\Altium\CM\Templates`, then restart CircuitMaker.

Apply the template to any open schematic by going to `Project-> Templates -> General Templates -> "A*_OSURC"`

If you haven't used the template in a project before, you will need to set the project parameters. You can access these by going to `Project -> Project Options -> Parameters`.

Edit the parameters so they look something like this:

![Parameters](http://nickmccomb.net/wp-content/uploads/2017/12/parameters.png "Parameters for Iris Project")

You'll also want to edit the `Title` parameter in the `Document Parameters` (found in `Project -> Document Options -> Parameters`) to reflect the title of your schematic document.

### Conventions

#### Project Names
`MR1718 Name`

#### Sheet Names
`lowercase_with_underscores`

#### Sheet Symbols
`UPPERCASE-DASHES`

#### Sheet Formatting
See [this](https://github.com/OSURoboticsClub/Rover_2017_2018/blob/master/electrical/schematics/schematic-example.pdf).

#### Power Net Naming
`voltage_descriptor` 

Where `descriptor` is either `RAW` or `SYS`. `voltage` is represented as shown in the example below.

`RAW` specifies any voltage that is not ready for system use, such as non-reverse-protected or non-current sensed portions of the circuit.

`SYS` is any voltage that is ready to be used by the system.

For example:

![power_net_naming](http://nickmccomb.net/wp-content/uploads/2017/12/power_net_naming_iris.png "Power Net Naming for the Iris Project")

### Exporting your Project

#### Board Outline
Ensure you have your board outline connector setup. You need an outline defined in the "Outline" layer (mechanical 1) as well as an exact copy on Mechanical 4 (per [CircuitMaker Fourm Post](https://circuitmaker.com/forum/posts/220409)).

We need both gerber files, and NC Drill files. Gerbers tell the manufacture where to put copper, silkscreen, etc, while the NC Drill files tell them where to drill. Your guess as to why they are different processes.

#### Outputting Gerbers

Make sure your file is 100% ready to go (do a final Design Rule Check), and then go to the output tab and select "Gerber". You will have to commit your project to export any manufacturing files. 

Choose the following settings:
![Gerber Settings 1](http://nickmccomb.net/wp-content/uploads/2018/01/2018-01-22-17_14_34-Gerber-Setup.png "Gerber Settings 1")
![Gerber Settings 2](http://nickmccomb.net/wp-content/uploads/2018/01/2018-01-22-17_14_55-Gerber-Setup.png "Gerber Settings 2")


Then hit OK, and save your project into the "Node Output Files" folder in the Google Drive (if outputting a node) under your folder's name.

#### Outputting NC Drill Files

Select "NC Drill Files" under the output menu. You will have to commit your project again.

Choose the following settings: 
![NC Drill Settings](http://nickmccomb.net/wp-content/uploads/2018/01/2018-01-22-17_18_01-NC-Drill-Setup.png "NC Drill Settings")

Save this .zip in the same folder as the gerber ones.

#### Assembling your files for being sent to the manufacturer

Extract both of the .zip files into their own folders. We're looking to establish the following group of files:

![Gerber Files](https://sites.google.com/a/oregonstate.edu/osurcknowledgebase/_/rsrc/1506362541118/engineering-resources/electrical-engineering/pcb-design/altium-designer-to-df-robot/2015-10-18%2000_43_59-OSH%20Park%20~%20Design%20Submission%20Guidelines.png "Gerber Files")

Make a folder with a basic board name (e.g. "IrisBoard") that will hold your finalized board generation files. This will be your staging folder.

Grab the **.TXT** from the "\*\_NC\_Drill" folder, rename it "_boardname_.XLN" and move it into your staging folder.

Rename the **.GM4** file from the "\_Gerber" folder to "_boardname_.GKO" and move it to your staging folder.

From the same folder, move the following files into your staging folder:

* .GBL
* .GBO
* .GBS
* .GTL
* .GTO
* .GTS

You should now have all the files from the picture above in your staging folder. Make a .zip of this folder. This folder is all you have to send to the manufacturer to make your board.

#### Verifying you've done this correctly

Upload your design to [OSHPark.com](https://oshpark.com/) and make sure that they render it correctly. This is a great first pass indicator to make sure you've done this process correctly. PCBWAY will also check your boards, but this prevents some dumb mistakes in assemling your .zip file.


#### Ordering your boards

Email the Team Lead with your .zip files, and he will place the order for you. Include the following information:

* Board dimensions in mm
* Desired color for the board (if not Red for Rover)
* Desired copper weight (1oz is the default, unless you have a reason for it to be different)





