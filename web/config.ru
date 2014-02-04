require 'json'

COMPILE_OPTIONS = {
  :peephole_optimization    =>true,
  :inline_const_cache       =>false,
  :specialized_instruction  =>false,
  :operands_unification     =>false,
  :instructions_unification =>false,
  :stack_caching            =>false,
}

def compile(source)
  RubyVM::InstructionSequence.compile(source, "src", "", 1, COMPILE_OPTIONS).to_a.to_json
end

run lambda{|env|
  if env['REQUEST_PATH'] == '/compileRuby.cgi' # TODO legacy url, gsub all js files to change it
    source = Rack::Request.new(env).params['src']
    [200, {'Content-Type'=>'text/plain'}, [compile(source)]]
  elsif env['REQUEST_PATH'] == '/'
    [200, {'Content-Type' => 'text/html'}, [File.read('index.html')]]
  else
    Rack::File.new('.').call(env)
  end
}
