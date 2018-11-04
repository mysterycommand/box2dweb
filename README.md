## Overview

This is a port of _Box2DFlash_ 2.1a to JavaScript. [We developed](http://hecht-software.de/projekte/forschung/) an ActionScript 3 -to- JavaScript compiler to generate the code.

There already exists a port to JavaScript called _Box2dJs_, but it's not up-to-date and you have to import a big amount of JavaScript files in every project, whereas my version is stored in a single file.

The _Box2D_ physics engine was developed by Erin Catto (visit http://www.gphysics.com for further information)

## Live Demo

<div>
<wiki:gadget url="http://box2dweb.googlecode.com/svn/trunk/demo-google-gadget.xml?v=5" height="420" width="620" border="0" /><br>
</div>

## Usage

You can read the documentation for _Box2dFlash_, since nearly everything is organized the same way.
http://www.box2dflash.org/docs/2.1a/reference/

The _b2DebugDraw_ takes a canvas-context instead of a Sprite:

```
var debugDraw = new Box2D.Dynamics.b2DebugDraw;
debugDraw.SetSprite(document.GetElementsByTagName("canvas")[0].getContext("2d"));
```

## Graphics

Please notice that Box2dWeb is a physics engine. The graphics in the demo are generated by the b2DebugDraw-class, which is only available for debugging purposes.
If you want to apply the computed coordinates to real graphics you should use a graphics library such as [IvanK](http://lib.ivank.net/) developed by Ivan Kuckir. The website of _IvanK_ even contains a Box2dWeb example: http://lib.ivank.net/index.php?p=demos&d=box2D
