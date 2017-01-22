from clang import cindex
import clang

def visitor(cursor, go=False):
    # if go:
        # print 'visiting', cursor.kind
    # if cursor.kind == clang.cindex.CursorKind.CALL_EXPR:

    children = list(cursor.get_children())
    # print '  has {0} children'.format(len(children))

    if cursor.kind == clang.cindex.CursorKind.FUNCTION_DECL:
        print 'Found %s [line=%s, col=%s]' % (
                cursor.displayname, cursor.location.line, cursor.location.column)
        for child in children:
            visitor(child, True)

    if cursor.kind == clang.cindex.CursorKind.PARM_DECL:
        print '\t', cursor.type.spelling, cursor.spelling

    for child in children:
        visitor(child)


if __name__ == '__main__':
    cindex.Config.set_library_path('/usr/lib/llvm-3.8/lib')
    path = '/home/jacob/repos/NVIDIA_CUDA-8.0_Samples/2_Graphics/volumeRender/volumeRender_kernel.cu'

    index = cindex.Index.create()
    tu = index.parse(path)

    visitor(tu.cursor)