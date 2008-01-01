#!/usr/local/bin/ruby
# Requires Ruby 1.9.0
# The license of this source is "Ruby License"

require 'json'
require 'cgi'

OutputCompileOption = {
  :peephole_optimization    =>true,
  :inline_const_cache       =>false,
  :specialized_instruction  =>false,
  :operands_unification     =>false,
  :instructions_unification =>false,
  :stack_caching            =>false,
}

cgi = CGI.new

puts "Content-type: text/plain\n\n"
puts VM::InstructionSequence.compile(cgi['src'], "src", 1, OutputCompileOption).to_a.to_json
