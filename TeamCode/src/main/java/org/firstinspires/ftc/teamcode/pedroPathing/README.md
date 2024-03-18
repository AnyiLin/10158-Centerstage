## Welcome!
This is the Pedro Pathing path following program developed by FTC team 10158 Scott's Bots in the 2023-2024 Centerstage season.

## Installation
The quickest way to get started is with the quickstart [here](https://www.google.com). REPLACE THIS

Otherwise, take the `pedroPathing` folder and put it under the `teamcode` folder in your project.
You can do this from either downloading the project from the above quickstart link or the 10158 CENTERSTAGE repository [here](https://github.com/brotherhobo/10158-Centerstage).

For this version of Pedro Pathing, the localizer used is the Road Runner localizer. To install its dependencies:
1. Find `build.dependencies.gradle` in the main folder of your project.
2. Add the following code to the end of the `repositories` block:
```
maven { = 'https://maven.brott.dev/' }
```
3. Then, add the following code to the end of your `dependencies` block:
```
implmentation 'com.acmerobotics.dashboard:dashboard:0.4.15'
```
4. Find the `build.gradle` file under the 'teamcode' folder.
5. In this gradle file, add the following dependencies:
```
implementation 'org.apache.commons:commons-math3:3.6.1'
implementation 'com.fasterxml/jackson.core:jacson-databind:2.12.7'
implementation 'com.acmerobotics.com.roadrunner:core:0.5.6'
```

/*probably create a separate overview .md file and a tuning .md file so our videos aren't strictly
 necessary but just additionally helpful. Ideally, we want Pedro Pathing to be accessible to as
 many people as possible, so making it easier to use without necessarily having to watch YouTube
 videos is better. However, linking the videos in the .md files will likely be best, since the
 videos will contain better and more in-depth explanations.
*/