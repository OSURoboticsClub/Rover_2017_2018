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
* rs485_termination
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

#### Sheet Formattinn
See [this](https://github.com/OSURoboticsClub/Rover_2017_2018/blob/master/electrical/schematics/schematic-example.pdf).

#### Power Net Naming
`voltage_descriptor` 

Where `descriptor` is either `RAW` or `SYS`. `voltage` is represented as shown in the example below.

`RAW` specifies any voltage that is not ready for system use, such as non-reverse-protected or non-current sensed portions of the circuit.

`SYS` is any voltage that is ready to be used by the system.

For example:

![power_net_naming](http://nickmccomb.net/wp-content/uploads/2017/12/power_net_naming_iris.png "Power Net Naming for the Iris Project")
