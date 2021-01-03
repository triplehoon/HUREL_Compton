using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HUREL.Compton
{

    class StringTest
    {
        static void Main()
        {
            // Create objects by using the new operator: 
            Child child1 = new Child("Craig", 11);
            Child child2 = new Child("Sally", 10);
            // Create an object using the default constructor:
            Child child3 = new Child("hello",5);
           

            // Display results:
            Console.Write("Child #1: ");
            child1.PrintChild();
            Console.Write("Child #2: ");
            child2.PrintChild();
            Console.Write("Child #3: ");
            child3.PrintChild();
        }
    }

    class Child2
    {
        public int Age;

    }

    class Child
    {
        private int age;
        private string name;

        private int Age;
        // Default constructor:
        public Child()
        {
            name = "N/A";
        }

        class StreamReader
        {

        }
        // Constructor: 
        public Child(string name, int age)
        {
            this.name = name;
            this.age = age;
            Age = age;
        }

        // Printing method: 
        public void PrintChild()
        {
            Console.WriteLine("{0}, {1} years old.", name, age);
        }
    }




}
