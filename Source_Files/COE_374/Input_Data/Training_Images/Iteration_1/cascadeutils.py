import os

def generate_negative_description_file():
    with open('neg.txt', 'w') as f:
        for filename in os.listdir('negative'):
            f.write('negative/' + filename + '\n')



def main():
    generate_negative_description_file()

if __name__ == "__main__":
    main()