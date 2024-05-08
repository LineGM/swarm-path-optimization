# ---- Утилиты для работы с Qt ----

# Функция получения пути к утилите qmake, на моей машине это "/usr/x86_64-w64-mingw32/lib/qt/bin/qmake"
function(get_qmake_executable_dir result)
	get_target_property(qmake_executable_dir Qt5::qmake IMPORTED_LOCATION)
	cmake_path(REMOVE_FILENAME qmake_executable_dir)
	if(qmake_executable_dir)
		message(DEBUG "qmake найден для целевой системы в папке: '${qmake_executable_dir}'")
	else(qmake_executable_dir)
		message(FATAL_ERROR "qmake не найден для целевой системы.")
	endif(qmake_executable_dir)
	set(${result}
		${qmake_executable_dir}
		PARENT_SCOPE)
endfunction(get_qmake_executable_dir)

#[===[Пример использования
get_qmake_executable_dir(result_value)
message(STATUS "Функция вернула: '${result_value}'")
]===]

# Функция получения пути к каталогу /plugins
function(get_qt_plugins_dir result)
	get_qmake_executable_dir(qt_plugins_dir)
	cmake_path(APPEND qt_plugins_dir "../plugins/")
	cmake_path(SET qt_plugins_dir NORMALIZE ${qt_plugins_dir})
	message(DEBUG "Сформирован путь к qt plugins: '${qt_plugins_dir}'.")
	set(${result}
		${qt_plugins_dir}
		PARENT_SCOPE)
endfunction(get_qt_plugins_dir)

#[===[Пример использования
get_qt_plugins_dir(result_value)
message(STATUS "Функция вернула: '${result_value}'")
]===]

# Функция получения пути к каталогу /platforms
function(get_qt_platforms_dir result)
	get_qmake_executable_dir(qt_platforms_dir)
	cmake_path(APPEND qt_platforms_dir "../plugins/platforms/")
	cmake_path(SET qt_platforms_dir NORMALIZE ${qt_platforms_dir})
	message(DEBUG "Сформирован путь к qt platforms: '${qt_platforms_dir}'.")
	set(${result}
		${qt_platforms_dir}
		PARENT_SCOPE)
endfunction(get_qt_platforms_dir)

#[===[Пример использования
get_qt_platforms_dir(result_value)
message(STATUS "Функция вернула: '${result_value}'")
]===]
