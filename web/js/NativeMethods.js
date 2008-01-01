// The license of this source is "Ruby License"
HotRuby.prototype.classes = {
	"<global>" : {
		"puts" : function(recver, args, sf) {
			if(args.length == 0) {
				this.printDebug("");
				return;
			}
			for(var i=0; i<args.length; i++) {
				var obj = args[i];
				if(obj == null) {
					this.printDebug("nil");
					continue;
				}
				if(typeof(obj) == "number" || typeof(obj) == "string") {
					this.printDebug(obj);
					continue;
				}
				if(obj instanceof Array) {
					for(var j=0; j<obj.length; j++) {
						this.printDebug(obj[j]);
					}
					continue;
				}
				
				var origSP = sf.sp;
				try {
					this.invokeMethod(obj, "to_ary", [], sf, 0, false);
					obj = sf.stack[--sf.sp];
					for(var j=0; j<obj.length; j++) {
						this.printDebug(obj[j]);
					}
					continue;
				} catch(e) {
				}
				sf.sp = origSP;

				this.invokeMethod(obj, "to_s", [], sf, 0, false);
				obj = sf.stack[--sf.sp];
				this.printDebug(obj);
			}
		}
	},

	"Object" : {
		"to_s" : function(recver) {
			return recver.toString();
		}
	},

	"TrueClass" : {
		"&" : function(recver, args) {
			return args[0] ? true : false;
		},
		
		"|" : function(recver, args) {
			return true;
		},

		"^" : function(recver, args) {
			return args[0] ? false : true;
		}
	},

	"FalseClass" : {
		"&" : function(recver, args) {
			return false;
		},
		
		"|" : function(recver, args) {
			return args[0] ? true : false;
		},

		"^" : function(recver, args) {
			return args[0] ? true : false;
		}
	},

	"NilClass" : {
	},

	"Proc" : {
		"yield" : function(recver, args, sf) {
			this.runOpcode(
				recver.__opcode, 
				recver.__parentStackFrame.classObj, 
				recver.__parentStackFrame.methodName, 
				recver.__parentStackFrame.self, 
				args, 
				recver.__parentStackFrame,
				true);
			return sf.stack[--sf.sp];
		}
	},

	"Float" : {
		"+" : function(recver, args) {
			return recver + args[0];
		},

		"-" : function(recver, args) {
			return recver - args[0];
		},

		"*" : function(recver, args) {
			return recver * args[0];
		},

		"/" : function(recver, args) {
			return recver / args[0];
		},
		
		"%" : function(recver, args) {
			return recver % args[0];
		},
		
		"<=>" : function(recver, args) {
			if(recver > args[0])
				return 1;
			else if(recver == args[0])
				return 0;
			if(recver < args[0])
				return -1;
		},
		
		"<" : function(recver, args) {
			return recver < args[0];
		},

		">" : function(recver, args) {
			return recver > args[0];
		},
		
		"<=" : function(recver, args) {
			return recver <= args[0];
		},

		">=" : function(recver, args) {
			return recver >= args[0];
		},
		
		"==" : function(recver, args) {
			return recver == args[0];
		},
		
		"times" : function(recver, args, sf) {
			for (var i = 0;i < recver; i++) {
				this.invokeMethod(args[0], "yield", [i], sf, 0, false);
				sf.sp--;
			}
		},
		
		"to_s" : function(recver) {
			return recver.toString();	
		}
	},

	"Integer" : {
		"+" : function(recver, args) {
			return recver + args[0];
		},

		"-" : function(recver, args) {
			return recver - args[0];
		},

		"*" : function(recver, args) {
			return recver * args[0];
		},

		"/" : function(recver, args) {
			return Math.floor(recver / args[0]);
		},
		
		"%" : function(recver, args) {
			return recver % args[0];
		},
		
		"<=>" : function(recver, args) {
			if(recver > args[0])
				return 1;
			else if(recver == args[0])
				return 0;
			if(recver < args[0])
				return -1;
		},
		
		"<" : function(recver, args) {
			return recver < args[0];
		},

		">" : function(recver, args) {
			return recver > args[0];
		},
		
		"<=" : function(recver, args) {
			return recver <= args[0];
		},

		">=" : function(recver, args) {
			return recver >= args[0];
		},
		
		"==" : function(recver, args) {
			return recver == args[0];
		}
	},

	"String" : {
		"+" : function(recver, args) {
			return this.createRubyString(recver + args[0]);
		},
		
		"*" : function(recver, args) {
			var ary = new Array(args[0]);
			for(var i=0; i<args[0]; i++) {
				ary[i] = recver;
			}
			return this.createRubyString(ary.join(""));
		},
		
		"==" : function(recver, args) {
			return recver == args[0];
		},
		
		"[]" : function(recver, args) {
			if(args.length == 1 && typeof(args[0]) == "number") {
				var no = args[0];
				if(no < 0) 
					no = recver.length + no;
				if(no < 0 || no >= recver.length)
					return null;
				return recver.charCodeAt(no);
			} else if(args.length == 2 && typeof(args[0]) == "number" && typeof(args[1]) == "number") {
				var start = args[0];
				if(start < 0) 
					start = recver.length + start;
				if(start < 0 || start >= recver.length)
					return null;
				if(args[1] < 0 || start + args[1] > recver.length)
					return null;
				return this.createRubyString(recver.substr(start, args[1]));
			} else {
				throw "Unsupported String[]";
			}
		}
	},
	
	"Array" : {
		"length" : function(recver) {
			return recver.length;
		},
		
		"size" : function(recver) {
			return recver.length;
		},
		
		"[]" : function(recver, args) {
			return recver[args[0]];
		},
		
		"[]=" : function(recver, args) {
			recver[args[0]] = args[1];
		},
		
		"join" : function(recver, args) {
			return this.createRubyString(recver.join(args[0]));
		},
		
		"to_s" : function(recver, args) {
			return this.createRubyString(recver.join(args[0]));
		}
	},
	
	"Hash" : {
		"[]" : function(recver, args) {
			return recver[args[0]];
		},
		
		"[]=" : function(recver, args) {
			if(!(args[0] in recver)) {
				recver.__instanceVars.length++;
			}
			return (recver[args[0]] = args[1]);
		},
		
		"length" : function(recver) {
			return recver.__instanceVars.length;
		},
		
		"size" : function(recver) {
			return recver.__instanceVars.length++;
		}
	},
	
	"Range" : {
		"each" : function(recver, args, sf) {
			if(recver.__instanceVars.exclude_end) {
				for (var i = recver.__instanceVars.first;i < recver.__instanceVars.last; i++) {
					this.invokeMethod(args[0], "yield", [i], sf, 0, false);
					sf.sp--;
				}
			} else {
				for (var i = recver.__instanceVars.first;i <= recver.__instanceVars.last; i++) {
					this.invokeMethod(args[0], "yield", [i], sf, 0, false);
					sf.sp--;
				}
			}
		},
		
		"begin" : function(recver) {
			return recver.__instanceVars.first;
		},
		
		"first" : function(recver) {
			return recver.__instanceVars.first;
		},
		
		"end" : function(recver) {
			return recver.__instanceVars.last;
		},
		
		"last" : function(recver) {
			return recver.__instanceVars.last;
		},
		
		"exclude_end?" : function(recver) {
			return recver.__instanceVars.exclude_end;
		},
		
		"length" : function(recver) {
			with(recver.__instanceVars) {
				return (last - first + (exclude_end ? 0 : 1));
			}
		},
		
		"size" : function(recver) {
			with(recver.__instanceVars) {
				return (last - first + (exclude_end ? 0 : 1));
			}
		},
		
		"step" : function(recver, args, sf) {
			var step;
			var proc;
			if(args.length == 1) { 
				step = 1;
				proc = args[0];
			} else {
				step = args[0];
				proc = args[1];
			}
			
			if(recver.__instanceVars.exclude_end) {
				for (var i = recver.__instanceVars.first;i < recver.__instanceVars.last; i += step) {
					this.invokeMethod(proc, "yield", [i], sf, 0, false);
					sf.sp--;
				}
			} else {
				for (var i = recver.__instanceVars.first;i <= recver.__instanceVars.last; i += step) {
					this.invokeMethod(proc, "yield", [i], sf, 0, false);
					sf.sp--;
				}
			}
		}
	},
	
	"Time" : {
		"initialize" : function(recver, args) {
			recver.__instanceVars.date = new Date(); 
		},
		
		"to_s" : function(recver) {
			return recver.__instanceVars.date.toString();
		},
		
		"to_f" : function(recver) {
			return recver.__instanceVars.date.getTime() / 1000;
		}
	}
};

(function() {
	var classes = HotRuby.prototype.classes;
	for (var className in classes) {
		classes[className].__className = className;
		classes[className].__parentClass = classes.Object;
		if(!("__constantVars" in classes[className]))
			classes[className].__constantVars = {};
		if(!("__classVars" in classes[className]))
			classes[className].__classVars = {};
	}
	
	classes.Object.__parentClass = classes["<global>"];
	classes["<global>"].__parentClass = null;
	
	for (var className in classes) {
		classes.Object.__constantVars[className] = classes[className];
	}
	delete classes.Object.__constantVars["<global>"];
})();
