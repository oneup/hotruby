class Foo
  CONST = 'Foo'
end

class Bar
  CONST = 'Bar'
  class Baz < Foo
    puts CONST             # => "Bar"      Outer class const
    # In this case, you have to specify if you want to see parent class const.
    puts Foo::CONST        # => "Foo"
  end
end

class Foo2
  CONST = 'Foo'
end

CONST = 'Object'

class Bar2 < Foo2
  puts CONST               # => "Foo"
end

# If you specify "Object", then const in Object is searched before.
class Object
  class Bar2 < Foo2
    puts CONST             # => "Object"
  end
end

class Foo3
  CONST = 'Foo'
end
class Bar3 < Foo3
  puts CONST               # => "Foo"
  CONST = 'Bar'            # Define Bar's const "CONST"
  puts CONST               # => "Bar"  (Foo::CONST is hidden)
  puts Foo3::CONST         # => "Foo"  (You can see by "::")
end
