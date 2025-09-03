# Bluetooth&reg; LE with CAPSENSE&trade; buttons and slider

This code example features a Bluetooth&reg; LE GATT Server with a 5-segment linear slider and two CAPSENSE&trade; buttons. Button 0 turns the LED ON, Button 1 turns the LED OFF, and the slider controls the brightness of the LED. The button status and slider data are sent to the GATT Client over GATT notification. This project uses the CAPSENSE&trade; middleware library; it is developed in the ModusToolbox&trade; software environment.

This code example uses FreeRTOS. visit the FreeRTOS website for documentation and API references.

[View this README on GitHub.](https://github.com/Infineon/mtb-example-btstack-freertos-capsense-buttons-slider)

[Provide feedback on this code example.](https://cypress.co1.qualtrics.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyMzAzMTciLCJTcGVjIE51bWJlciI6IjAwMi0zMDMxNyIsIkRvYyBUaXRsZSI6IkJsdWV0b290aCZyZWc7IExFIHdpdGggQ0FQU0VOU0UmdHJhZGU7IGJ1dHRvbnMgYW5kIHNsaWRlciIsInJpZCI6InNoYWhzaHViaGFtcyIsIkRvYyB2ZXJzaW9uIjoiNi4xLjAiLCJEb2MgTGFuZ3VhZ2UiOiJFbmdsaXNoIiwiRG9jIERpdmlzaW9uIjoiTUNEIiwiRG9jIEJVIjoiSUNXIiwiRG9jIEZhbWlseSI6IlBTT0MifQ==)

## Requirements

- [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) v3.1 or later (tested with v3.1)
- Board support package (BSP) minimum required version: 4.0.0
- Programming language: C
- Associated parts: All [PSoC&trade; 6 MCU](https://www.infineon.com/cms/en/product/microcontroller/32-bit-psoc-arm-cortex-microcontroller/psoc-6-32-bit-arm-cortex-m4-mcu) parts, [AIROC&trade; CYW20819 Bluetooth&reg; & Bluetooth&reg; LE SoC](https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-bluetooth-le-bluetooth-multiprotocol/airoc-bluetooth-le-bluetooth/cyw20819), [AIROC&trade; CYW43012 Wi-Fi & Bluetooth&reg; combo chip](https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-wi-fi-plus-bluetooth-combos/wi-fi-4-802.11n/cyw43012), [AIROC&trade; CYW4343W Wi-Fi & Bluetooth&reg; combo chip](https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-wi-fi-plus-bluetooth-combos/wi-fi-4-802.11n/cyw4343w), [AIROC&trade; CYW4373 Wi-Fi & Bluetooth&reg; combo chip](https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-wi-fi-plus-bluetooth-combos/wi-fi-5-802.11ac/cyw4373), [AIROC&trade; CYW43439 Wi-Fi & Bluetooth&reg; combo chip](https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-wi-fi-plus-bluetooth-combos/wi-fi-4-802.11n/cyw43439)


## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v11.3.1 (`GCC_ARM`) - Default value of `TOOLCHAIN`
- Arm&reg; Compiler v6.16 (`ARM`)
- IAR C/C++ Compiler v9.30.1 (`IAR`)

## Supported kits (make variable 'TARGET')

- [PSoC&trade; 6 Wi-Fi Bluetooth&reg; Pioneer Kit](https://www.infineon.com/CY8CKIT-062-WIFI-BT) (`CY8CKIT-062-WIFI-BT`) - Default value of `TARGET`
- [PSoC&trade; 6 Wi-Fi Bluetooth&reg; Prototyping Kit](https://www.infineon.com/CY8CPROTO-062-4343W) (`CY8CPROTO-062-4343W`)
- [PSoC&trade; 62S2 Wi-Fi Bluetooth&reg; Pioneer Kit](https://www.infineon.com/CY8CKIT-062S2-43012) (`CY8CKIT-062S2-43012`)
- [PSoC&trade; 62S1 Wi-Fi Bluetooth&reg; Pioneer Kit](https://www.infineon.com/CYW9P62S1-43438EVB-01) (`CYW9P62S1-43438EVB-01`)
- [PSoC&trade; 62S1 Wi-Fi Bluetooth&reg; Pioneer Kit](https://www.infineon.com/CYW9P62S1-43012EVB-01) (`CYW9P62S1-43012EVB-01`)
- [PSoC&trade; 62S3 Wi-Fi Bluetooth&reg; Prototyping Kit](https://www.infineon.com/CY8CPROTO-062S3-4343W) (`CY8CPROTO-062S3-4343W`)
- [PSoC&trade; 62S2 Evaluation Kit](https://www.infineon.com/CY8CEVAL-062S2) (`CY8CEVAL-062S2`, `CY8CEVAL-062S2-LAI-4373M2`,`CY8CEVAL-062S2-CYW43022CUB`,
`CY8CEVAL-062S2-CYW955513SDM2WLIPA`)
- [PSoC&trade; 6 Bluetooth&reg; LE Pioneer Kit](https://www.infineon.com/CY8CKIT-062-BLE) (`CY8CKIT-062-BLE`)

## Hardware setup
This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.

> **Note:** The PSoC&trade; 6 Bluetooth&reg; LE Pioneer Kit (CY8CKIT-062-BLE) and the PSoC&trade; 6 Wi-Fi Bluetooth&reg; Pioneer Kit (CY8CKIT-062-WIFI-BT) ship with KitProg2 installed. ModusToolbox&trade; requires KitProg3. Before using this code example, make sure that the board is upgraded to KitProg3. The tool and instructions are available in the [Firmware Loader](https://github.com/Infineon/Firmware-loader) GitHub repository. If you do not upgrade, you will see an error like "unable to find CMSIS-DAP device" or "KitProg firmware is out of date".


## Software setup

See the [ModusToolbox&trade; tools package installation guide](https://www.infineon.com/ModusToolboxInstallguide) for information about installing and configuring the tools package.

Download and install the AIROC&trade; Bluetooth&reg; Connect App on your [Android](https://play.google.com/store/apps/details?id=com.infineon.airocbluetoothconnect) or [iOS](https://apps.apple.com/in/app/airoc-bluetooth-connect-app/id6443702288) phone.

Install a terminal emulator if you don't have one. Instructions in this document use [Tera Term](https://teratermproject.github.io/index-en.html).




## Using the code example

### Create the project

The ModusToolbox&trade; tools package provides the Project Creator as both a GUI tool and a command line tool.

<details><summary><b>Use Project Creator GUI</b></summary>

1. Open the Project Creator GUI tool.

   There are several ways to do this, including launching it from the dashboard or from inside the Eclipse IDE. For more details, see the [Project Creator user guide](https://www.infineon.com/ModusToolboxProjectCreator) (locally available at *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/docs/project-creator.pdf*).

2. On the **Choose Board Support Package (BSP)** page, select a kit supported by this code example. See [Supported kits](#supported-kits-make-variable-target).

   > **Note:** To use this code example for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work.

3. On the **Select Application** page:

   a. Select the **Applications(s) Root Path** and the **Target IDE**.

   > **Note:** Depending on how you open the Project Creator tool, these fields may be pre-selected for you.

   b.	Select this code example from the list by enabling its check box.

   > **Note:** You can narrow the list of displayed examples by typing in the filter box.

   c. (Optional) Change the suggested **New Application Name** and **New BSP Name**.

   d. Click **Create** to complete the application creation process.

</details>

<details><summary><b>Use Project Creator CLI</b></summary>

The 'project-creator-cli' tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the 'project-creator-cli' tool. On Windows, use the command-line 'modus-shell' program provided in the ModusToolbox&trade; installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; tools. You can access it by typing "modus-shell" in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.

The following example clones the "[mtb-example-btstack-freertos-capsense-buttons-slider](https://github.com/Infineon/mtb-example-btstack-freertos-capsense-buttons-slider)" application with the desired name "MyCapsenseButtonsSlider" configured for the *CY8CKIT-062-WIFI-BT* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id CY8CKIT-062-WIFI-BT --app-id mtb-example-btstack-freertos-capsense-buttons-slider --user-app-name MyCapsenseButtonsSlider --target-dir "C:/mtb_projects"
   ```


The 'project-creator-cli' tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the <id> field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the <id> field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional

> **Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For details, see the "Project creator tools" section of the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at {ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf).

</details>



### Open the project

After the project has been created, you can open it in your preferred development environment.


<details><summary><b>Eclipse IDE</b></summary>

If you opened the Project Creator tool from the included Eclipse IDE, the project will open in Eclipse automatically.

For more details, see the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_ide_user_guide.pdf*).

</details>


<details><summary><b>Visual Studio (VS) Code</b></summary>

Launch VS Code manually, and then open the generated *{project-name}.code-workspace* file located in the project directory.

For more details, see the [Visual Studio Code for ModusToolbox&trade; user guide](https://www.infineon.com/MTBVSCodeUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_vscode_user_guide.pdf*).

</details>


<details><summary><b>Keil µVision</b></summary>

Double-click the generated *{project-name}.cprj* file to launch the Keil µVision IDE.

For more details, see the [Keil µVision for ModusToolbox&trade; user guide](https://www.infineon.com/MTBuVisionUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_uvision_user_guide.pdf*).

</details>


<details><summary><b>IAR Embedded Workbench</b></summary>

Open IAR Embedded Workbench manually, and create a new project. Then select the generated *{project-name}.ipcf* file located in the project directory.

For more details, see the [IAR Embedded Workbench for ModusToolbox&trade; user guide](https://www.infineon.com/MTBIARUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_iar_user_guide.pdf*).

</details>


<details><summary><b>Command line</b></summary>

If you prefer to use the CLI, open the appropriate terminal, and navigate to the project directory. On Windows, use the command-line 'modus-shell' program; on Linux and macOS, you can use any terminal application. From there, you can run various `make` commands.

For more details, see the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>



## Operation


If using a PSoC&trade; 64 "Secure" MCU kit (like CY8CKIT-064B0S2-4343W), the PSoC&trade; 64 device must be provisioned with keys and policies before being programmed. Follow the instructions in the ["Secure Boot" SDK user guide](https://www.infineon.com/dgdlac/Infineon-PSoC_64_Secure_MCU_Secure_Boot_SDK_User_Guide-Software-v07_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0f8c361a7666) to provision the device. If the kit is already provisioned, copy-paste the keys and policy folder to the application folder.

1. Connect the board to your PC using the provided USB cable through the KitProg3 USB connector.

2. Open a terminal program and select the KitProg3 COM port. Set the serial port parameters to 8N1 and 115200 baud.

3. Program the board using one of the following:

   <details><summary><b>Using Eclipse IDE</b></summary>

      1. Select the application project in the Project Explorer.

      2. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (KitProg3_MiniProg4)**.
   </details>


   <details><summary><b>In other IDEs</b></summary>

   Follow the instructions in your preferred IDE.
   </details>


   <details><summary><b>Using CLI</b></summary>

     From the terminal, execute the `make program` command to build and program the application using the default toolchain to the default target. The default toolchain is specified in the application's Makefile but you can override this value manually:
      ```
      make program TOOLCHAIN=<toolchain>
      ```

      Example:
      ```
      make program TOOLCHAIN=GCC_ARM
      ```
   </details>

4. After programming, the application starts automatically. Observe the messages on the UART terminal, and wait for the device to make all the required connections. Use the KitProg3 COM port to view the Bluetooth&reg; stack and application trace messages in the terminal window as shown in **Figure 1**.

   **Figure 1. Log messages on KitProg3 COM port**

   ![](images/figure2.png)

5. To test the application, touch the CAPSENSE&trade; button 1 (BTN1) to turn the LED OFF, touch Button 0 (BTN0) to turn the LED ON, and touch the slider in different positions to change the brightness.

6. To test using the AIROC&trade; Bluetooth&reg; Connect App using the mobile, do the following (see equivalent AIROC&trade; Bluetooth&reg; Connect App screenshots in **Figure 2** and **Figure 3**):

    1. Turn ON Bluetooth&reg; on your Android or iOS device.

    2. Launch the AIROC&trade; Bluetooth&reg; Connect App.

    3. Press the reset switch on the CY8CKIT-062-WiFi-BT kit to start sending advertisements. The red LED (LED1) starts glowing. Touch the slider to change the LED brightness. Advertising will stop after 90 seconds if a connection has not been established.

    4. Swipe down on the AIROC&trade; Bluetooth&reg; Connect App home screen to start scanning for Bluetooth&reg; LE Peripherals; your device (**CAPSENSE&trade; Button Slider**) appears in the AIROC&trade; Bluetooth&reg; Connect App home screen. Select your device to establish a Bluetooth&reg; LE connection.

       Two Services are shown in the AIROC&trade; Bluetooth&reg; Connect carousel view.

    6. Select the **CAPSENSE&trade; Slider** profile from the services. Touch the slider and change its position on the board. It changes the slider graphic pattern on the app screen. In addition, the brightness of the LED changes based on the slider position.

    7. Select the **CAPSENSE&trade; Buttons** profile from the services. Touch Button 0 and Button 1 to change the button graphic on the app screen; the LED status toggles ON/OFF.


**Figure 2** and **Figure 3** show the steps for using the AIROC&trade; Bluetooth&reg; Connect App on iOS and Android respectively.

**Figure 2. Testing with the AIROC&trade; Bluetooth&reg; Connect App on iOS**

![](images/figure1a.png)


**Figure 3. Testing with the AIROC&trade; Bluetooth&reg; Connect App on Android**

![](images/figure1b.png)

7. You can also monitor the CAPSENSE&trade; data using the CAPSENSE&trade; Tuner application as follows:

   **Monitor data using CAPSENSE&trade; Tuner**

    1. Open CAPSENSE&trade; Tuner from the IDE Quick Panel.

        You can also run the CAPSENSE&trade; Tuner application standalone from *{ModusToolbox&trade; software install directory}/ModusToolbox/tools_{version}/capsense-configurator/capsense-tuner*. In this case, after opening the application, select **File** > **Open** and open the *design.cycapsense* file for the respective kit, which is present in the *TARGET_\<BSP-NAME>/{version}/COMPONENT_BSP_DESIGN_MODUS* or *COMPONENT_CUSTOM_DESIGN_MODUS* folder.

    2. Ensure that the kit is in KitProg3 mode. See [Firmware-loader](https://github.com/Infineon/Firmware-loader) to learn on how to update the firmware and switch to KitProg3 mode.

    3. In the tuner application, click **Tuner Communication Setup** or select **Tools** > **Tuner Communication Setup**. In the window that appears, select the I2C checkbox under KitProg3 and configure as follows:

       - **I2C Address:** 8
       - **Sub-address:** 2-Bytes
       - **Speed (kHz):** 400

    5. Click **Connect**, or select **Communication** > **Connect**.

    6. Click **Start**, or select **Communication** > **Start**.

    Under the **Widget View** tab, you can see the corresponding widgets highlighted in blue color when you touch the button or slider. You can also view the sensor data in the **Graph View** tab. For example, to view the sensor data for Button 0, select **Button0_Sns0** under **Button0**.

    **Figure 4** shows the CAPSENSE&trade; Tuner displaying the status of a touch on Button 0 and LinearSlider 0.

    **Figure 4. CAPSENSE&trade; Tuner showing touch data**

    ![](images/tuner_display.png)

    The CAPSENSE&trade; Tuner can also be used for tuning CAPSENSE&trade; parameters and measuring signal-to-noise ratio (SNR). See the [ModusToolbox&trade; CAPSENSE&trade; Tuner user guide](https://www.infineon.com/ModusToolboxCapSenseTuner) (**Help** > **View Help**) and [AN85951 – PSoC&trade; 4 and PSoC&trade; 6 MCU CAPSENSE&trade; design guide](https://www.infineon.com/AN85951) for more details on selecting the tuning parameters.


## Debugging


You can debug the example to step through the code.


<details><summary><b>In Eclipse IDE</b></summary>

Use the **\<Application Name> Debug (KitProg3_MiniProg4)** configuration in the **Quick Panel**. For details, see the "Program and debug" section in the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide).


> **Note:** **(Only while debugging for PSoC&trade; 6 MCU)** On the CM4 CPU, some code in `main()` may execute before the debugger halts at the beginning of `main()`. This means that some code executes twice – once before the debugger stops execution, and again after the debugger resets the program counter to the beginning of `main()`. See [KBA231071](https://community.infineon.com/docs/DOC-21143) to learn about this and for the workaround.

</details>


<details><summary><b>In other IDEs</b></summary>

Follow the instructions in your preferred IDE.

</details>



## Design and implementation

The Bluetooth&reg; LE CAPSENSE&trade; Buttons and Slider is a Bluetooth&reg; LE GATT Server where the CAPSENSE&trade; buttons and slider data is stored and notified to the client; it scans a self-capacitance (CSD) based, 5-element CAPSENSE&trade; slider and two mutual capacitance (CSX) CAPSENSE&trade; buttons for user input.

See [AN85951 – PSoC&trade; 4 and PSoC&trade; 6 MCU CAPSENSE&trade; design guide](https://www.infineon.com/AN85951) for more details on CAPSENSE&trade; features and usage.

In this application, the state of the user LED is controlled based on user inputs provided using the CAPSENSE&trade; buttons and slider. A PWM HAL resource is configured for controlling the brightness of the LED. Touching Button 0 turn the LED ON and Button 1 turn the LED OFF. The brightness of the LED is set based on the touch position on the CAPSENSE&trade; slider.

The [ModusToolbox&trade; CAPSENSE&trade; Configurator user guide](https://www.infineon.com/ModusToolboxCapSenseConfig) provides step-by-step instructions on how to configure CAPSENSE&trade; in the application. The CAPSENSE&trade; Configurator and Tuner tools can be launched from the CSD personality in the Device Configurator tool.

The application uses an [EZI2C HAL](https://infineon.github.io/mtb-hal-cat1/html/group__group__hal__ezi2c.html) interface for communicating with the CAPSENSE&trade; Tuner.

The firmware uses FreeRTOS to execute the tasks required by this application. The following tasks are created:

1. **CAPSENSE&trade; task:** Initializes the CAPSENSE&trade; hardware block, processes the touch input, and sends a command to the LED task to update the LED status.

2. **Board task:** Initializes the TCPWM in PWM mode for driving the LED and updates the status of the LED based on the received command.

3. **Bluetooth&reg; LE task:** Initializes the Bluetooth&reg; stack and GATT services, and sends the CAPSENSE&trade; buttons and slider values to the client via notifications.

A FreeRTOS-based timer is used for making the CAPSENSE&trade; scan periodic; a queue is used for communication between the CAPSENSE&trade; task and LED task. *FreeRTOSConfig.h* contains the FreeRTOS settings and configurations.

**Figure 5. CAPSENSE&trade; buttons and slider implementation**

![](images/figure3.png)


### Operation at custom power supply voltages

The application is configured to work with the default operating voltage of the kit.

**Table 1. Operating voltages supported by the kits**

 Kit                   | Supported operating voltages | Default operating voltage
 :-------------------- | ---------------------------- | -------------------------
 CY8CPROTO-062-4343W   | 3.3 V/1.8 V                  | 3.3 V                     
 CY8CKIT-062-WIFI-BT   | 3.3 V/1.8 V                  | 3.3 V                     
 CY8CKIT-062S2-43012   | 3.3 V/1.8 V                  | 3.3 V                     
 CYW9P62S1-43438EVB-01 | 3.3 V only                   | 3.3 V                     
 CYW9P62S1-43012EVB-01 | 1.8 V only                   | 1.8 V                     
 CY8CKIT-062-BLE       | 3.3 V/1.8 V                  | 3.3 V                     

<br>

For kits that support multiple operating voltages, the default BSP configuration provided by the *design.modus* file should be customized. Do the following to use the example at a custom power supply, such as 1.8 V:

1. Create a directory at the root of the application to hold any custom BSP configuration files: *\<application_folder>/COMPONENT_CUSTOM_DESIGN_MODUS*.

2. Create a subdirectory for each target that needs to be modified to work at a custom power supply: *\<application_folder>/COMPONENT_CUSTOM_DESIGN_MODUS/TARGET_\<kit>*.

3. Copy the *design.modus* file and other configuration files (from the *../mtb_shared/TARGET_\<kit>/{version}/COMPONENT_BSP_DESIGN_MODUS*) directory and paste them into the new directory for the target.

4. In the application's Makefile, update the following lines:
   ```
   DISABLE_COMPONENTS += BSP_DESIGN_MODUS
   COMPONENTS += CUSTOM_DESIGN_MODUS
   ```
5. Launch the [Device Configurator](https://www.infineon.com/ModusToolboxDeviceConfig) tool using the Quick Panel link in the IDE. This opens the *design.modus* file from the newly created *COMPONENT_CUSTOM_DESIGN_MODUS/TARGET_\<kit>* folder; you are now free to customize the configuration as required.

6. Update the **Operating Conditions** parameters in **Power Settings** with the desired voltage and select **File** > **Save**.

   **Figure 6. Power settings to work with 1.8 V**

   ![](images/figure4.png)

7. Change the jumper/switch setting as follows:

   **Table 2. Jumper/switch position for 1.8-V operation**

    Kit                    | Jumper/switch position 
    :----------------------| --------------------- 
    CY8CPROTO-062-4343W    | J3 (1-2)               
    CY8CKIT-062-WIFI-BT    | SW5 (1-2)              
    CY8CKIT-062S2-43012    | J14 (1-2)              
    CY8CPROTO-062S3-4343W  | J3 (1-2)               
    CY8CPROTO-062S3-4343W  | SW5 (1-2)              

<br>

8. Re-build and program the application to evaluate the application at the new power setting.


### Resources and settings

- **Device Configurator:** ModusToolbox&trade; stores the device configuration settings of the application in the *design.modus* file. This file is used by the Device Configurator which generate the configuration firmware. This file is stored in the respective kit BSP folder in *mtb_shared* project. For example, for CY8CKIT-062S2-43012 the path is *<workspace_path>mtb_shared\TARGET_CY8CKIT-062S2-43012\latest-v3.X\COMPONENT_BSP_DESIGN_MODUS/design.modus*.
See the
[Device configurator guide](https://www.infineon.com/ModusToolboxDeviceConfig).

- **Bluetooth&reg; Configurator:** The Bluetooth&reg; peripheral has an additional configurator called the “Bluetooth&reg; Configurator” that is used to generate the Bluetooth&reg; LE GATT database and various Bluetooth&reg; settings for the application. These settings are stored in the file named *design.cybt*. Note that unlike the Device Configurator, the Bluetooth&reg; Configurator settings and files are local to each respective application. As explained in the [Design and implementation](#design-and-implementation) section, the only extra service added is the Immediate Alert Service (IAS).
 See the [Bluetooth&reg; Configurator guide](https://www.infineon.com/ModusToolboxBLEConfig).

**Table 3. Application resources**

  File name  |  comments  |
-----------------------------------|-------------------------------------------------------
*main.c* | Contains the functions to create FreeRTOS tasks for CAPSENSE&trade;, Bluetooth&reg; LE, and LED modules. This is the entry point for execution of the user application code after device startup.
*cycfg_gatt_db.c, cycfg_gatt_db.h*| Files in the *GeneratedSource* folder under the application folder. They contain the GATT database information generated using the Bluetooth&reg; Configurator tool.
*bt_app.c, bt_app.h*| Contain the functions for the Bluetooth&reg; stack event handler functions.
*capsense.c, capsense.h*| Contain the functions for CAPSENSE&trade; processing and handler functions.
*board.c, board.h*| Contain the functions for handle the LED with PWM.

## Related resources

Resources  | Links
-----------|----------------------------------
Application notes  | [AN228571](https://www.infineon.com/AN228571) – Getting started with PSoC&trade; 6 MCU on ModusToolbox&trade; <br>  [AN215656](https://www.infineon.com/AN215656) – PSoC&trade; 6 MCU: Dual-CPU system design
Code examples  | [Using ModusToolbox&trade;](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub
Device documentation | [PSoC&trade; 6 MCU datasheets](https://documentation.infineon.com/html/psoc6/bnm1651211483724.html) <br> [PSoC&trade; 6 technical reference manuals](https://documentation.infineon.com/html/psoc6/zrs1651212645947.html)
Development kits | Select your kits from the [Evaluation board finder](https://www.infineon.com/cms/en/design-support/finder-selection-tools/product-finder/evaluation-board).
Libraries on GitHub  | [mtb-pdl-cat1](https://github.com/Infineon/mtb-pdl-cat1) – PSoC&trade; 6 Peripheral Driver Library (PDL)  <br> [mtb-hal-cat1](https://github.com/Infineon/mtb-hal-cat1) – Hardware Abstraction Layer (HAL) library <br> [retarget-io](https://github.com/Infineon/retarget-io) – Utility library to retarget STDIO messages to a UART port
Middleware on GitHub  | [capsense](https://github.com/Infineon/capsense) – CAPSENSE&trade; library and documents <br> [psoc6-middleware](https://github.com/Infineon/modustoolbox-software#psoc-6-middleware-libraries) – Links to all PSoC&trade; 6 MCU middleware
Tools  | [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) – ModusToolbox&trade; software is a collection of easy-to-use libraries and tools enabling rapid development with Infineon MCUs for applications ranging from wireless and cloud-connected systems, edge AI/ML, embedded sense and control, to wired USB connectivity using PSoC&trade; Industrial/IoT MCUs, AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices, XMC&trade; Industrial MCUs, and EZ-USB&trade;/EZ-PD&trade; wired connectivity controllers. ModusToolbox&trade; incorporates a comprehensive set of BSPs, HAL, libraries, configuration tools, and provides support for industry-standard IDEs to fast-track your embedded application development.

<br>



## Other resources


Infineon provides a wealth of data at [www.infineon.com](https://www.infineon.com) to help you select the right device, and quickly and effectively integrate it into your design.

For PSoC&trade; 6 MCU devices, see [How to design with PSoC&trade; 6 MCU - KBA223067](https://community.infineon.com/docs/DOC-14644) in the Infineon Developer community.


## Document history


Document title: *CE230317* – *Bluetooth&reg; LE with CAPSENSE&trade; buttons and slider*

| Version | Description of change 
| ------- | --------------------- 
| 1.0.0   | New code example      
| 2.0.0   | Major update to support ModusToolbox&trade; v2.2, added support for new kits <br> This version is not backward compatible with ModusToolbox&trade; v2.1
| 3.0.0   | Updated to support ModusToolbox&trade; v2.3 and and it is not backward compatible with v2.2 and earlier
| 4.0.0   | Added support for 43439 kit <br> Updated BSP to 3.0.0
| 5.0.0   | Updated to support ModusToolbox&trade; v3.0 and BSPs v4.X 
| 5.1.0   | Updated the CAPSENSE&trade; library version to 3.X 
| 5.1.1   | Readme update 
| 5.1.2   | Added support for CY8CEVAL-062S2-CYW43022CUB
| 6.0.0   | CAPSENSE&trade; version update to 4.X. <br> Added BSP support for CY8CKIT-062-BLE; Bluetooth&reg; stack update and code improvements
|6.1.0    | Added support for CY8CEVAL-062S2-CYW955513SDM2WLIPA

------

All referenced product or service names and trademarks are the property of their respective owners.

The Bluetooth&reg; word mark and logos are registered trademarks owned by Bluetooth SIG, Inc., and any use of such marks by Infineon is under license.


---------------------------------------------------------

© Cypress Semiconductor Corporation, 2020-2024. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").  This document, including any software or firmware included or referenced in this document ("Software"), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress's patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
<br>
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  No computing device can be absolutely secure.  Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product. CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, "Security Breach").  Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach.  In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications. To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document. Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes.  It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product.  "High-Risk Device" means any device or system whose failure could cause personal injury, death, or property damage.  Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices.  "Critical Component" means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness.  Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device. You shall indemnify and hold Cypress, including its affiliates, and its directors, officers, employees, agents, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device. Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress's published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
<br>
Cypress, the Cypress logo, and combinations thereof, ModusToolbox, PSoC, CAPSENSE, EZ-USB, F-RAM, and TRAVEO are trademarks or registered trademarks of Cypress or a subsidiary of Cypress in the United States or in other countries. For a more complete list of Cypress trademarks, visit www.infineon.com. Other names and brands may be claimed as property of their respective owners.
