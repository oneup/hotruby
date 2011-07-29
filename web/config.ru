require 'json'

compile_options = {
  :peephole_optimization    =>true,
  :inline_const_cache       =>false,
  :specialized_instruction  =>false,
  :operands_unification     =>false,
  :instructions_unification =>false,
  :stack_caching            =>false,
}

run lambda{|env|
  source = Rack::Request.new(env).GET['src']
  compiled = RubyVM::InstructionSequence.compile(source, "src", "", 1, compile_options).to_a.to_json
  [200, {'Content-Type'=>'text/plain'}, [compiled]]
}
