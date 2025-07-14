import pymysql

conn = pymysql.connect(
    host='localhost',
    user='root',
    password='password',
    db='mydatabase',
    charset='utf8mb4',
    cursorclass=pymysql.cursors.DictCursor
)


try:
    with conn.cursor() as cursor:
        # Create a new record
        sql = "INSERT INTO `users` (`email`, `password`) VALUES (%s, %s)"
        cursor.execute(sql, ('john@example.com', 'mypassword'))

    # Commit changes
    conn.commit()

    print("Record inserted successfully")
finally:
    conn.close()

