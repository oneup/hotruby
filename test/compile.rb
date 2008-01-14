require 'optparse'
require 'pp'
require 'json'

OutputCompileOption = {
  :peephole_optimization    =>true,
  :inline_const_cache       =>false,
  :specialized_instruction  =>false,
  :operands_unification     =>false,
  :instructions_unification =>false,
  :stack_caching            =>false,
}

def compile_to_json infile, outfile, prettyfile
  iseq = VM::InstructionSequence.compile_file(infile, OutputCompileOption)

  open(outfile, 'w'){|f|
    f.puts "var src = " + iseq.to_a.to_json + ";"
  }
  if prettyfile
	open(prettyfile, 'w'){|f|
		f.puts JSON::pretty_generate(iseq.to_a, :indent => "\t")
	}
  end
end

## main

outfile = ''
prettyfile = nil
opt = OptionParser.new{|opt|
  opt.on('-o file'){|o|
    outfile = o
  }
  opt.on('-p file'){|o|
    prettyfile = o
  }
  opt.version = '0.0.1'
}

opt.parse!(ARGV)

ARGV.each{|file|
  case outfile
  when /\.js\Z/
    compile_to_json file, outfile, prettyfile
  else
    raise
  end
}

