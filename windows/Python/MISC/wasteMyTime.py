# look for perfect numbers where the sum of numbers its divisible by are equal to the number
#include <stdio.h>


def func(num):
    while True:
        num += 1
        sum = 0
        for i in range( 1, num): #count to num but not including num
            if (num % i == 0):
                sum = sum + i
        if (sum == num):
            print(num)   



def main():
    # Your code here
    print("Hello, World!")

    num = 1

    func(num)
    
if __name__ == "__main__":
    main()