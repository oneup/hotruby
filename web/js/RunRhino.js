/*
 * Usage:
 * java -jar "C:\Program Files\Java\rhino1_6R7\js.jar" RunRhino.js test\Const.js
 */
load("HotRuby.js", "NativeMethods.js", arguments[0]);
new HotRuby().run(src);