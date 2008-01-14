// The license of this source is "Ruby License"
(function() {

var asNativeClasses = {
	"ASEnviornment" : {
	},
	"ASObject" : {
	}
};
for(var className in asNativeClasses) {
	HotRuby.prototype.classes[className] = asNativeClasses[className];
}

})();

