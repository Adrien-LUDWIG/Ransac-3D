#include <kdtree.h>
#include <obj.h>

using namespace tnp;

int main(int argc, char *argv[])
{
    // option -----------------------------------------------------------------
    if(argc <= 1) {
        std::cout << "Error: missing filename" << std::endl;
        return 1;
    }
    const auto filename = std::string(argv[1]);

    // load -------------------------------------------------------------------
    auto points = std::vector<Eigen::Vector3f>();
    auto normals = std::vector<Eigen::Vector3f>();
    if(not tnp::load_obj(filename, points, normals))
    {
        std::cout << "Error: failed to open input file '" << filename << "'" << std::endl;
        return 1;
    }

    // process ----------------------------------------------------------------
    // ...

    return 0;
}
