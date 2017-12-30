## How to CircuitMaker for OSURC
----

### Install
Download and install CircuitMaker from [here](https://workspace.circuitmaker.com/Account/SignUp).

### Starting a New Project 
Start a new project and make sure to use the naming convention when making the title, you can't change it later -_-. We use `MR1718 [Name]`. For example, the motor node would be `MR1718 Motor`. To make things uniform there are generated project images you can download from the drive, so add that on the main project page as well.

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

Download the sheet template file and add it to `C:\Users\Public\Documents\Altium\CM\Templates`, then restart CircuitMaker.

### Conventions

Project Names: `MR1718 Name`

Sheet Names: `lowercase_with_underscores`

Sheet Symbols: `UPPERCASE-DASHES`

Sheet Formatting: See [this](https://github.com/OSURoboticsClub/Rover_2017_2018/blob/master/electrical/schematics/schematic-example.pdf).