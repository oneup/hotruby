// The license of this source is "Ruby License"
/**
 * HotRuby
 * @class
 * @construtor
 */
var HotRuby = function() {
	/** 
	 * Global Variables
	 * @type Object 
	 */
	this.globalVars = {};
	/** 
	 * END blocks
	 * @type Array 
	 */
	this.endBlocks = [];
	/**
	 * Running Enviornment
	 * @type String
	 */
	this.env = "browser";
	this.nilObj = {
		__className : "NilClass",
		__parentClass : this.classes.Object
	};
	this.trueObj = {
		__className : "TrueClass",
		__parentClass : this.classes.Object
	};
	this.falseObj = {
		__className : "FalseClass",
		__parentClass : this.classes.Object
	};
	
	this.checkEnv();
};

/**
 * StackFrame
 * @class
 * @construtor
 */
HotRuby.StackFrame = function() {
	/** 
	 * Stack Pointer
	 * @type Number 
	 */
	this.sp = 0;
	/** 
	 * Local Variables
	 * @type Array 
	 */
	this.localVars = [];
	/** 
	 * Stack 
	 * @type Array 
	 */
	this.stack = [];
	/** 
	 * Current class to define methods
	 * @type Object 
	 */
	this.classObj = null;
	/** 
	 * Current method name
	 * @type String 
	 */
	this.methodName = "";
	/** 
	 * Current line no
	 * @type Number 
	 */
	this.lineNo = 0;
	/** 
	 * File name
	 * @type String 
	 */
	this.fileName = "";
	/** 
	 * self
	 * @type Object 
	 */
	this.self = null;
	/** 
	 * Parent StackFrame
	 * @type HotRuby.StackFrame 
	 */
	this.parentStackFrame = null;
	/** 
	 * Is Proc(Block)
	 * @type boolean 
	 */
	this.isProc = false;
	/** 
	 * Object Specific class
	 * @type Object 
	 */
	this.cbaseObj = null;
};

HotRuby.prototype = {
	/**
	 * Run the script.
 	 * @param {Array} opcode
	 */
	run : function(opcode) {
		try {
			this.runOpcode(opcode, this.classes["<global>"], null, null, [], null, false, null);
		} catch(e) {
			alert(e);
		}
	},
	
	/**
	 * Run the opcode.
	 * @param {Array} opcode
	 * @param {Object} classObj
	 * @param {String} methodName
	 * @param {Object} self
	 * @param {Array} args
	 * @param {HotRuby.StackFrame} parentSF Parent StackFrame
	 * @param {boolean} isProc
	 * @param {Object} cbaseObj
	 * @private
	 */
	runOpcode : function(opcode, classObj, methodName, self, args, parentSF, isProc, cbaseObj) {
		if(args.length < opcode[4].arg_size)
			throw "[runOpcode] Wrong number of arguments (" + args.length + " for " + opcode[4].arg_size + ")";
		
		// Create Stack Frame
		var sf = new HotRuby.StackFrame();
		sf.localVars = new Array(opcode[4].local_size + 1);
		sf.stack = new Array(opcode[4].stack_max);
		sf.fileName = opcode[6];
		sf.classObj = classObj;
		sf.methodName = methodName;
		sf.self = self;
		sf.parentStackFrame = parentSF;
		sf.isProc = isProc;
		sf.cbaseObj = cbaseObj;
		
		// Copy args to localVars. Fill from last.
		for (var i = 0;i < opcode[4].arg_size; i++) {
			sf.localVars[sf.localVars.length - 1 - i] = args[i];
		}
		
		// Run the mainLoop
		this.mainLoop(opcode[11], sf);
		
		// Copy the stack to the parent stack frame
		if (parentSF != null) {
			for (var i = 0;i < sf.sp; i++) {
				parentSF.stack[parentSF.sp++] = sf.stack[i];
			}
		} else {
			// Run END blocks
			if(this.endBlocks.length > 0) {
				this.run(this.endBlocks.pop());
			}
		}
	},
	
	/**
	 * Main loop for opcodes.
	 * @param {Array} opcode
	 * @param {HotRuby.StackFrame} sf
	 * @private
	 */
	mainLoop : function(opcode, sf) {
		// Create label to ip
		if(!("label2ip" in opcode)) {
			opcode.label2ip = {};
			for (var ip = 0;ip < opcode.length; ip++) {
				// If "cmd is a String then it is a jump label
				var cmd = opcode[ip];
				if (typeof(cmd) == "string") {
					opcode.label2ip[cmd] = ip;
					opcode[ip] = null;
				}
			}
		}
		
		for (var ip = 0;ip < opcode.length; ip++) {
			// Get the next command
			var cmd = opcode[ip];
			if (cmd == null)
				continue;

			// If "cmd" is a Number then it is the line number.
			if (typeof(cmd) == "number") {
				sf.lineNo = cmd;
				continue;
			}
			// "cmd" must be an Array
			if (!(cmd instanceof Array))
				continue;
			
			//console.log("cmd = " + cmd[0] + ", sp = " + sf.sp);
			switch (cmd[0]) {
				case "jump" :
					ip = opcode.label2ip[cmd[1]];
					break;
				case "branchif" :
					var val = sf.stack[--sf.sp];
					if(val != this.nilObj && val != this.falseObj) {
						ip = opcode.label2ip[cmd[1]];
					}
					break;
				case "branchunless" :
					var val = sf.stack[--sf.sp];
					if(val == this.nilObj || val == this.falseObj) {
						ip = opcode.label2ip[cmd[1]];
					}
					break;
				case "leave" :
					return;
				case "putnil" :
					sf.stack[sf.sp++] = this.nilObj;
					break;
				case "putself" :
					sf.stack[sf.sp++] = sf.self;
					break;
				case "putobject" :
					var value = cmd[1];
					if(typeof(value) == "string") {
						if(value.match(/^(\d+)\.\.(\d+)$/)) {
							value = this.createRubyRange(
								parseInt(RegExp.$2), 
								parseInt(RegExp.$1), 
								false);
						}
					}
					sf.stack[sf.sp++] = value;
					break;
				case "putstring" :
					sf.stack[sf.sp++] = this.createRubyString(cmd[1]);
					break;
				case "concatstrings" :
					sf.stack[sf.sp++] = this.createRubyString(
						sf.stack.slice(sf.stack.length - cmd[1], sf.stack.length).join());
					break;
				case "newarray" :
					var value = this.createRubyArray(sf.stack.slice(sf.sp - cmd[1], sf.sp));
					sf.sp -= value.__native.length;
					sf.stack[sf.sp++] = value;
					break;
				case "duparray" :
					sf.stack[sf.sp++] = this.createRubyArray(cmd[1]);
					break;
				case "expandarray" :
					var ary = sf.stack[--sf.sp];
					if(typeof(ary) == "object" && ary.__className == "Array") {
						for(var i=0; i<cmd[1]; i++) {
							sf.stack[sf.sp++] = ary.__native[i];						
						}
						if(cmd[2] && 1) {
							// TODO
						}
						if(cmd[2] && 2) {
							// TODO
						}
						if(cmd[2] && 4) {
							// TODO
						}
					} else {
						sf.stack[sf.sp++] = ary;
						for (var i = 0;i < cmd[1] - 1; i++) {
							sf.stack[sf.sp++] = this.nilObj;
						}
					}
					break;
				case "newhash" :
					var hash = this.createRubyHash(sf.stack.slice(sf.sp - cmd[1], sf.sp));
					sf.sp -= cmd[1];
					sf.stack[sf.sp++] = hash;
					break;
				case "newrange" :
					var value = this.createRubyRange(sf.stack[--sf.sp], sf.stack[--sf.sp], cmd[1]);
					sf.stack[sf.sp++] = value;
					break;
				case "setlocal" :
					var localSF = sf;
					while (localSF.isProc) {
						localSF = localSF.parentStackFrame;
					}
					localSF.localVars[cmd[1]] = sf.stack[--sf.sp];
					break;
				case "getlocal" :
					var localSF = sf;
					while (localSF.isProc) {
						localSF = localSF.parentStackFrame;
					}
					sf.stack[sf.sp++] = localSF.localVars[cmd[1]];
					break;
				case "setglobal" :
					this.globalVars[cmd[1]] = sf.stack[--sf.sp];
					break;
				case "getglobal" :
					sf.stack[sf.sp++] = this.globalVars[cmd[1]];
					break;
				case "setconstant" :
					this.setConstant(sf, sf.stack[--sf.sp], cmd[1], sf.stack[--sf.sp]);
					break;
				case "getconstant" :
					var value = this.getConstant(sf, sf.stack[--sf.sp], cmd[1]);
					sf.stack[sf.sp++] = value;
					break;
				case "setinstancevariable" :
					sf.self.__instanceVars[cmd[1]] = sf.stack[--sf.sp];
					break;
				case "getinstancevariable" :
					sf.stack[sf.sp++] = sf.self.__instanceVars[cmd[1]];
					break;
				case "setclassvariable" :
					sf.classObj.__classVars[cmd[1]] = sf.stack[--sf.sp];
					break;
				case "getclassvariable" :
					var searchClass = sf.classObj;
					while (true) {
						if (cmd[1] in searchClass.__classVars) {
							sf.stack[sf.sp++] = searchClass.__classVars[cmd[1]];
							break;
						}
						searchClass = searchClass.__parentClass;
						if (searchClass == null) {
							throw "Cannot find class variable : " + cmd[1];
						}
					}
					break;
				case "getdynamic" :
					var lookupSF = sf;
					for (var i = 0;i < cmd[2]; i++) {
						lookupSF = lookupSF.parentStackFrame;
					}
					sf.stack[sf.sp++] = lookupSF.localVars[cmd[1]];
					break;
				case "setdynamic" :
					var lookupSF = sf;
					for (var i = 0;i < cmd[2]; i++) {
						lookupSF = lookupSF.parentStackFrame;
					}
					lookupSF.localVars[cmd[1]] = sf.stack[--sf.sp];
					break;
//				case "getspecial" :
//					break;
//				case "setspecial" :
//					break;
				case "pop" :
					sf.sp--;
					break;
				case "dup" :
					sf.stack[sf.sp] = sf.stack[sf.sp - 1];
					sf.sp++;
					break;
				case "dupn" :
					for (var i = 0;i < cmd[1]; i++) {
						sf.stack[sf.sp + i] = sf.stack[sf.sp + i - cmd[1]];
					}
					sf.sp += cmd[1];
					break;
				case "swap" :
					var tmp = sf.stack[sf.sp - 1];
					sf.stack[sf.sp - 1] = sf.stack[sf.sp - 2];
					sf.stack[sf.sp - 2] = tmp;
					break;
				case "topn" :
					sf.stack[sf.sp] = sf.stack[sf.sp - cmd[1]];
					sf.sp++;
					break;
				case "setn" :
					sf.stack[sf.sp - cmd[1]] = sf.stack[sf.sp - 1];
					break;
				case "emptstack" :
					sf.sp = 0;
					break;
				case "send" :
					var args = sf.stack.slice(sf.sp - cmd[2], sf.sp);
					sf.sp -= cmd[2];
					var recver = sf.stack[--sf.sp];
					if (recver == null || recver == this.nilObj)
						recver = sf.self;
					if(cmd[3] instanceof Array)
						cmd[3] = this.createRubyProc(cmd[3], sf);
					if(cmd[3] != null)
						args.push(cmd[3]);
					this.invokeMethod(recver, cmd[1], args, sf, cmd[4], false);
					break;
				case "invokesuper" :
					var args = sf.stack.slice(sf.sp - cmd[1], sf.sp);
					sf.sp -= cmd[1];
					// TODO When to use this autoPassAllArgs?
					var autoPassAllArgs = sf.stack[--sf.sp];
					if(cmd[2] instanceof Array)
						cmd[2] = this.createRubyProc(cmd[1], sf);
					if(cmd[2] != null)
						args.push(cmd[2]);
					this.invokeMethod(sf.self, sf.methodName, args, sf, cmd[3], true);
					break;
				case "definemethod" :
					var obj = sf.stack[--sf.sp];
					if(sf.cbaseObj != null)
						obj = sf.cbaseObj;
					if (obj == null || obj == this.nilObj) {
						sf.classObj[cmd[1]] = cmd[2];
					} else {
						if (!("__methods" in obj))
						//if(typeof(obj.__methods) == "undefined")
							obj.__methods = {};
						obj.__methods[cmd[1]] = cmd[2];
					}
					opcode[ip] = null;
					opcode[ip - 1] = null;
					break;
				case "defineclass" :
					var parentClass = sf.stack[--sf.sp];
					var isRedefine = (parentClass == this.falseObj);
					if(parentClass == null || parentClass == this.nilObj)
						parentClass = this.classes.Object;
					var cbaseObj = sf.stack[--sf.sp];
					if(cmd[3] == 0) {
						// Search predefined class
						var newClass = this.getConstant(sf, sf.classObj, cmd[1]);
						if(newClass == null || isRedefine) {
							// Create class object
							var newClass = {
								__className : cmd[1],
								__parentClass : parentClass,
								__constantVars : {},
								__classVars : {}
							};
							this.classes[cmd[1]] = newClass;
							// Puts the className to CONSTANT
							this.setConstant(sf, sf.classObj, cmd[1], newClass);
						}
						// Run the class definition
						this.runOpcode(cmd[2], newClass, null, null, [], sf, false, null);
					} else if(cmd[3] == 1) {
						// Object-Specific Classes
						if(cbaseObj == null || typeof(cbaseObj) != "object")
							throw "Not supported Object-Specific Classes on Primitive Object"
						// Run the class definition
						this.runOpcode(cmd[2], cbaseObj.__className, null, null, [], sf, false, cbaseObj);
					} else 	if(cmd[3] == 2) {
						// TODO 
						throw "Not implemented";
					}
					break;
				case "postexe" :
					this.endBlocks.push(cmd[1]);
					break;
				case "nop" :
					break;
				case "reput" :
					break;
				default :
					throw "[mainLoop] Unknown opcode : " + cmd[0];
			}
		}
	},
	
	/**
	 * Invoke the method
	 * @param {Object} recver
	 * @param {String} methodName
	 * @param {Array} args
	 * @param {HotRuby.StackFrame} sf
	 * @param {Number} type VM_CALL_ARGS_SPLAT_BIT, ...
	 * @param {boolean} invokeSuper
	 */
	invokeMethod : function(recver, methodName, args, sf, type, invokeSuper) {
		var recverClassName = this.getClassName(recver);
		var invokeClassName = recverClassName;
		var invokeMethodName = methodName;
		var func = null;
		
		if (invokeSuper) {
			var searchClass = this.classes[recverClassName];
			while (func == null) {
				// Search Parent class
				if (!("__parentClass" in searchClass)) break;
				searchClass = searchClass.__parentClass;
				invokeClassName = searchClass.__className;

				// Search method in class
				func = searchClass[methodName];
			}
		} else {
			// Search method in object
			//if (recver != null && recver.__methods != null) {
			if (recver != null && typeof(recver) == "object" && "__methods" in recver) {
				func = recver.__methods[methodName];
			}
			if (func == null) {
				var searchClass = this.classes[recverClassName];
				while (true) {
					// Search method in class
					func = searchClass[methodName];
					if (func != null) break;
					
					if (methodName == "new") {
						func = searchClass["initialize"];
						if (func != null) {
							invokeMethodName = "initialize";
							break;
						}
					}
	
					// Search Parent class
					if ("__parentClass" in searchClass) {
						searchClass = searchClass.__parentClass;
						if(searchClass == null) {
							func = null;
							break;
						}
						invokeClassName = searchClass.__className;
						continue;
					}
					break;
				}
			}
		}
		if (func == null) {
			if (invokeSuper) {
				sf.stack[sf.sp++] = null;
				return;
			}
			if (methodName != "new") {
				throw "[invokeMethod] Undefined function : " + methodName;
			}
		}
		
		if (methodName == "new") {
			// Create instance
			var newObj = {
				__className : recverClassName,
				__instanceVars : {}
			};
			sf.stack[sf.sp++] = newObj;
			if (func == null) return;

			recver = newObj;
		}

		// Splat array args
		if (type & HotRuby.VM_CALL_ARGS_SPLAT_BIT) {
			args = args.concat(args.pop().__native);
		}
		
		// Exec method
		switch (typeof(func)) {
			case "function" :
				sf.stack[sf.sp++] = func.call(this, recver, args, sf);
				break;
			case "object" :
				this.runOpcode(func, this.classes[invokeClassName],
						invokeMethodName, recver, args, sf, false, sf.cbaseObj);
				break;
			default :
				throw "[invokeMethod] Unknown function type : " + typeof(func);
		}
		
		// Returned value of initialize() is unnecessally at new()
		if (methodName == "new") {
			sf.sp--;
		}
	},
	
	/**
	 * Set the Constant
	 * @param {HotRuby.StackFrame} sf
	 * @param {Object} classObj
	 * @param {String} constName
	 * @param constValue
	 * @private
	 */
	setConstant : function(sf, classObj, constName, constValue) {
		if (classObj == null || classObj == this.nilObj) {
			classObj = sf.classObj;
		} else if (classObj == false || classObj == this.falseObj) {
			// TODO
			throw "[setConstant] Not implemented";
		}
		// Const in <global> belongs to Object
		if(classObj.__className == "<global>")
			classObj = this.classes.Object;
		classObj.__constantVars[constName] = constValue;
	},
	
	/**
	 * Get the constant
	 * @param {HotRuby.StackFrame} sf
	 * @param {Object} classObj
	 * @param {String} constName
	 * @return constant value
	 * @private
	 */
	getConstant : function(sf, classObj, constName) {
		if (classObj == null || classObj == this.nilObj) {
			var isFound = false;
			// Search outer(parentStackFrame)
			for (var checkSF = sf;!isFound; checkSF = checkSF.parentStackFrame) {
				if (checkSF.classObj.__className == "<global>") {
					break;
				}
				if (constName in checkSF.classObj.__constantVars) {
					classObj = checkSF.classObj;
					isFound = true;
				}
			}
			// Search parent class
			if (!isFound) {
				for (classObj = sf.classObj;classObj.__className != "<global>";) {
					if (constName in classObj.__constantVars) {
						isFound = true;
						break;
					}
					classObj = classObj.__parentClass;
				}
			}
			// Search in Object class
			if (!isFound) {
				classObj = this.classes.Object;
			}
		} else if (classObj == false || classObj == this.falseObj) {
			// TODO
			throw "[setConstant] Not implemented";
		}
		if (classObj == null || classObj == this.nilObj)
			throw "[getConstant] Cannot find constant : " + constName;
		// Const in <global> belongs to Object
		if (classObj.__className == "<global>")
			classObj = this.classes.Object;
		return classObj.__constantVars[constName];
	},
	
	/**
	 * Returns class name from object.
	 * @param obj
	 * @return {String}
	 */
	getClassName : function(obj) {
		if (obj == null)
			return "<global>";
		switch (typeof(obj)) {
			case "object" :
				return obj.__className;
			case "number" :
				return "Float";
//			case "string" :
//				return "String";
//			case "boolean" :
//				return obj ? "TrueClass" : "FalseClass";
			default :
				throw "[getClassName] unknown type : " + typeof(obj);
		}
	},
	
	/**
	 * JavaScript String -> Ruby String
	 * @param {String} str
	 * @return {String}
	 */
	createRubyString : function(str) {
		return {
			__native : str,
			__className : "String",
			__parentClass : this.classes.Object
		};
	},
	
	/**
	 * opcode -> Ruby Proc
	 * @param {Array} opcode
	 * @param {HotRuby.StackFrame} sf
	 * @return {Object} Proc
	 */
	createRubyProc : function(opcode, sf) {
		return {
			__opcode : opcode,
			__className : "Proc",
			__parentClass : this.classes.Object,
			__parentStackFrame : sf
		};
	},
	
	/**
	 * JavaScript Array -> Ruby Array
	 * @param {Array} ary
	 * @return {Array}
	 */
	createRubyArray : function(ary) {
		return {
			__native : ary,
			__className : "Array",
			__parentClass : this.classes.Object
		};
	},
	
	/**
	 * JavaScript Array -> Ruby Hash
	 * @param {Array} ary
	 * @return {Object}
	 */
	createRubyHash : function(ary) {
		var hash = {
			__className : "Hash",
			__parentClass : this.classes.Object,
			__instanceVars : {
				length : ary.length / 2
			},
			__native : {}
		};
		for (var i = 0;i < ary.length; i += 2) {
			if(typeof(ary[i]) == "object" && ary[i].__className == "String") {
				hash.__native[ary[i].__native] = ary[i + 1];
			} else {
				throw "[createRubyHash] Unsupported. Cannot put this object to Hash";
			}
		}
		return hash;
	},
	
	/**
	 * Creates Ruby Range
	 * @param {Number} last
	 * @param {Number} first
	 * @param {boolean} exclude_end
	 */
	createRubyRange : function(last, first, exclude_end) {
		return {
			__className : "Range",
			__parentClass : this.classes.Object,
			__instanceVars : {
				first : first,
				last : last,
				exclude_end : exclude_end ? this.trueObj : this.falseObj
			}
		};
	},
	
	/**
	 * Print to debug dom.
	 * @param {String} str
	 */
	printDebug : function(str) {
		switch(HotRuby.env) {
			case "browser":
				var div = document.createElement("div");
				var text = document.createTextNode(str);
				div.appendChild(text);
				this.debugDom.appendChild(div);
				break;
			case "flash":
	            HotRuby.debugTextField.text += str + "\n";
				break;
			case "rhino":
				print(str);
				break;
		}
	},

	/**
	 * Search <script type="text/ruby"></script> and run.
	 * @param {String} url Ruby compiler url
	 */
	runFromScriptTag : function(url) {
		var ary = document.getElementsByTagName("script");
		for(var i=0; i < ary.length; i++) {
			var hoge = ary[i].type;
			if(ary[i].type == "text/ruby") {
				this.compileAndRun(url, ary[i].text);
				break;
			}
		}
	},
	
	/**
	 * Send the source to server and run.
	 * @param {String} url Ruby compiler url
	 * @param {src} Ruby source
	 */
	compileAndRun : function(url, src) {
		Ext.lib.Ajax.request(
			"POST",
			url,
			{
				success: function(response) {
					if(response.responseText.length == 0) {
						alert("Compile failed");
					} else {
						this.run(eval("(" + response.responseText + ")"));
					}
				},
				failure: function(response) {
					alert("Compile failed");
				},
				scope: this
			},
			"src=" + encodeURIComponent(src)
		);
	},
	
	checkEnv : function() {
		if(typeof(_root) != "undefined") {
			HotRuby.env = "flash";
            HotRuby.debugTextField = new TextField();
            HotRuby.debugTextField.autoSize = TextFieldAutoSize.LEFT;
            _root.addChild(HotRuby.debugTextField);
			alert = function(str) {
				HotRuby.debugTextField.text += str + "\n";
			}
		} else if(typeof(alert) == "undefined") {
			HotRuby.env = "rhino";
			alert = function(str) {
				print(str);
			}
		} else {
			HotRuby.env = "browser";
			this.debugDom = document.getElementById("debug");
			if (this.debugDom == null) {
				this.debugDom = document.body;
			}
		}
	}
};

// Consts
/** @memberof HotRuby */
HotRuby.VM_CALL_ARGS_SPLAT_BIT = 2;
/** @memberof HotRuby */
HotRuby.VM_CALL_ARGS_BLOCKARG_BIT = 4;
/** @memberof HotRuby */
HotRuby.VM_CALL_FCALL_BIT = 8;
/** @memberof HotRuby */
HotRuby.VM_CALL_VCALL_BIT = 16;

