task :build do
  File.open(File.join('web','js','HotRuby.js'),'w') do |f|
    f.write File.read(File.join('src','RubyVM.js')) +
            File.read(File.join('src','RubyNative.js'))
  end
end
