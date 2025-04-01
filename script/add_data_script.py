import sys


def generate_datas_macros(n: int, file_name: str):
    file = open(file_name, "w")
    file.write("#pragma once\n\n")
    for i in range(n + 1):
        # print(f'PARSE_DATA({i:03d});\n', end = "")
        # file.write(f'PARSE_DATA({(i):03d});\n')
        file.write(f'inline constexpr char DATA_{i:03d}[] = {{\n' +
                   f'\t#embed "../datas/velodyne_points/data/0000000{i:03d}.txt"\n' +
                   f'}};\n\n')
    # combine into one big array
    file.write(f'inline std::array<const char*, {n + 1}> DATA= {{\n')
    for i in range(n + 1):
        file.write(f'\tDATA_{i:03d}')
        if i != n:
            file.write(",\n")

    file.write(f'\n}};\n')
    file.close()


#"datas/velodyne_points/data/0000000" #num ".txt"

def main():
    generate_datas_macros(110, "../include/datas.hpp")


if __name__ == "__main__":
    main()
